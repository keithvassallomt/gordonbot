from __future__ import annotations
import io
import os
import time
import threading
from typing import Generator, Optional
import logging

from PIL import Image  # type: ignore

try:
    # Picamera2 is normally installed via apt: `sudo apt install -y python3-picamera2`
    from picamera2 import Picamera2  # type: ignore
    _PICAM_AVAILABLE = True
except Exception as e:
    # Common case on Raspberry Pi when using a venv: apt packages live in
    # /usr/lib/python3/dist-packages and are not visible to the venv by default.
    log = logging.getLogger(__name__)
    log.info("Picamera2 import failed in venv: %s — trying system dist-packages", e)
    try:
        import sys, os
        added = False
        for p in ("/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"):
            if os.path.isdir(p) and p not in sys.path:
                sys.path.append(p)
                added = True
        if added:
            from picamera2 import Picamera2  # type: ignore
            _PICAM_AVAILABLE = True
        else:
            Picamera2 = None  # type: ignore
            _PICAM_AVAILABLE = False
            log.info("No system dist-packages path found to load Picamera2")
    except Exception as e2:
        Picamera2 = None  # type: ignore
        _PICAM_AVAILABLE = False
        log.info("Picamera2 still unavailable after path injection: %s", e2)


class Camera:
    """Minimal camera abstraction producing JPEG frames.

    - Uses Picamera2 when available (on Raspberry Pi)
    """

    def __init__(self, width: int = 640, height: int = 480, quality: int = 80, fps: int = 10) -> None:
        self.width = width
        self.height = height
        self.quality = quality
        self.fps = fps
        self._picam: Optional[Picamera2] = None  # type: ignore
        self._lock = threading.Lock()
        self._open_lock = threading.Lock()
        self._start_time = time.time()
        # Buffers for latest frames
        self._last_frame_main = None  # type: ignore
        self._last_frame_lores = None  # type: ignore
        # Some platforms deliver "RGB888" as BGR in memory; allow swapping.
        # Set CAMERA_SWAP_RB=0 to disable if colors look wrong after this change.
        self._swap_rb = os.environ.get("CAMERA_SWAP_RB", "1") not in ("0", "false", "False")
        # Publishing state
        self._publishing = False
        self._publish_lock = threading.Lock()
        self._ffmpeg_proc = None

        # Defer opening Picamera2 until first use (capture or publisher)
        if not _PICAM_AVAILABLE:
            raise RuntimeError("Picamera2 not available; install python3-picamera2 or run on the device.")

    def close(self) -> None:
        with self._lock:
            if self._picam is not None:
                try:
                    self._picam.stop()
                except Exception:
                    pass
                self._picam = None

    def _ensure_open(self) -> None:
        if self._picam is not None:
            return
        with self._open_lock:
            if self._picam is not None:
                return
            try:
                cam = Picamera2()  # type: ignore
                cfg = cam.create_video_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"},
                    lores={"size": (max(160, self.width // 4), max(120, self.height // 4)), "format": "YUV420"},
                )
                cam.configure(cfg)

                def _on_frame(request):  # type: ignore
                    try:
                        try:
                            arr_lo = request.make_array("lores")
                            self._last_frame_lores = arr_lo
                        except Exception:
                            pass
                        try:
                            arr_main = request.make_array("main")
                            self._last_frame_main = arr_main
                        except Exception:
                            pass
                    except Exception:
                        pass

                cam.post_callback = _on_frame  # type: ignore
                cam.start()
                self._picam = cam
            except Exception as e:
                self._picam = None
                raise RuntimeError("Failed to initialize Picamera2") from e

    def _latest_frame(self):
        # Return latest frame array and whether it's RGB (True) or YUV (False)
        with self._lock:
            picam = self._picam
            last_main = self._last_frame_main
        if picam is None:
            # Open lazily on first use
            self._ensure_open()
            with self._lock:
                picam = self._picam
                last_main = self._last_frame_main
        if last_main is not None:
            return last_main, True
        try:
            arr = picam.capture_array()  # type: ignore[union-attr]
            with self._lock:
                self._last_frame_main = arr
            return arr, True
        except Exception:
            return None, True

    def latest_arrays(self) -> tuple[object | None, object | None]:
        """Return a tuple of (main, lores) numpy arrays if available.

        Returned objects are numpy ndarrays or None when not yet captured.
        """
        with self._lock:
            return self._last_frame_main, self._last_frame_lores

    def capture_jpeg(self) -> bytes:
        # Use latest frame if available; otherwise raise to surface issues
        frame, is_rgb = self._latest_frame()
        if frame is None:
            raise RuntimeError("No frame available from camera")
        try:
            if is_rgb:
                # Ensure RGB channel order for PIL; swap if needed
                try:
                    if self._swap_rb and getattr(frame, "ndim", 0) == 3 and frame.shape[2] == 3:
                        img = Image.fromarray(frame[:, :, ::-1])  # BGR -> RGB
                    else:
                        img = Image.fromarray(frame)
                except Exception:
                    img = Image.fromarray(frame)
            else:
                raise RuntimeError("Unsupported frame format (YUV) for capture_jpeg")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=self.quality)
            return buf.getvalue()
        except Exception as e:
            raise RuntimeError("Failed to encode JPEG from camera frame") from e

    def mjpeg_generator(self) -> Generator[bytes, None, None]:
        # multipart/x-mixed-replace generator yielding JPEG parts
        delay = max(0.0, 1.0 / max(1, self.fps))
        while True:
            frame = self.capture_jpeg()
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
            time.sleep(delay)

    # ---- Publisher to MediaMTX (RTSP) using Picamera2 encoder -----------------
    def start_publisher(self, rtsp_url: str, bitrate: int = 2_000_000) -> bool:
        """Publish H.264 to RTSP via ffmpeg pipe for reliability.

        Avoids Picamera2 FfmpegOutput autodetection issues with URL sinks.
        """
        if not _PICAM_AVAILABLE:
            logging.getLogger(__name__).warning("start_publisher: Picamera2 not available")
            return False
        with self._publish_lock:
            if self._publishing:
                return True
            try:
                # Ensure camera is open before starting encoder
                self._ensure_open()
                from picamera2.encoders import H264Encoder  # type: ignore
                from picamera2.outputs import FileOutput  # type: ignore
                import subprocess, shlex

                # Launch ffmpeg to read raw H.264 from stdin and push to RTSP
                cmd = [
                    "ffmpeg", "-loglevel", "warning", "-re",
                    "-f", "h264", "-i", "-",
                    "-c", "copy",
                    "-f", "rtsp", rtsp_url,
                ]
                self._ffmpeg_proc = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
                )
                if not self._ffmpeg_proc or not self._ffmpeg_proc.stdin:
                    logging.getLogger(__name__).error("Failed to start ffmpeg subprocess for RTSP publish")
                    return False

                encoder = H264Encoder(bitrate=bitrate)
                sink = FileOutput(self._ffmpeg_proc.stdin)
                self._picam.start_recording(encoder, sink, name="main")
                self._publishing = True
                logging.getLogger(__name__).info("Started publishing to %s", rtsp_url)
                return True
            except FileNotFoundError:
                logging.getLogger(__name__).error("ffmpeg not found; install it with 'sudo apt install -y ffmpeg'")
                self._publishing = False
                return False
            except Exception as e:
                logging.getLogger(__name__).exception("Failed to start publisher: %s", e)
                self._publishing = False
                # Ensure ffmpeg is torn down if partially started
                try:
                    if self._ffmpeg_proc:
                        self._ffmpeg_proc.kill()
                except Exception:
                    pass
                self._ffmpeg_proc = None
                return False

    def stop_publisher(self) -> None:
        if not _PICAM_AVAILABLE or self._picam is None:
            return
        with self._publish_lock:
            if not self._publishing:
                return
            try:
                self._picam.stop_recording()
            except Exception:
                pass
            try:
                if self._ffmpeg_proc:
                    self._ffmpeg_proc.stdin.close()  # type: ignore[union-attr]
                    self._ffmpeg_proc.terminate()
            except Exception:
                pass
            self._ffmpeg_proc = None
            self._publishing = False


# Singleton camera instance (1080p)
camera = Camera(width=1920, height=1080, fps=15, quality=85)
