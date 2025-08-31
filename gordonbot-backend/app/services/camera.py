from __future__ import annotations
import io
import time
import threading
from typing import Generator, Optional
import logging

from PIL import Image, ImageDraw, ImageFont  # type: ignore

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
    - Falls back to a generated placeholder frame off-device
    """

    def __init__(self, width: int = 640, height: int = 480, quality: int = 80, fps: int = 10) -> None:
        self.width = width
        self.height = height
        self.quality = quality
        self.fps = fps
        self._picam: Optional[Picamera2] = None  # type: ignore
        self._lock = threading.Lock()
        self._start_time = time.time()

        if _PICAM_AVAILABLE:
            try:
                cam = Picamera2()  # type: ignore
                cfg = cam.create_video_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"}
                )
                cam.configure(cfg)
                cam.start()
                self._picam = cam
            except Exception:
                # Fall back to placeholder if camera init fails
                self._picam = None
                logging.getLogger(__name__).warning("Picamera2 present but failed to init; using placeholder frames.")
        else:
            logging.getLogger(__name__).info("Picamera2 not available; using placeholder frames.")

    def close(self) -> None:
        with self._lock:
            if self._picam is not None:
                try:
                    self._picam.stop()
                except Exception:
                    pass
                self._picam = None

    def _placeholder_frame(self) -> bytes:
        # Simple generated image with timestamp for dev machines
        img = Image.new("RGB", (self.width, self.height), (20, 20, 20))
        draw = ImageDraw.Draw(img)
        t = time.time() - self._start_time
        msg = f"GordonBot Camera\nDEV PLACEHOLDER\n{t:6.2f}s"
        # Use default font; keep it legible
        try:
            font = ImageFont.load_default()
        except Exception:
            font = None  # type: ignore
        draw.rectangle([(10, 10), (self.width - 10, self.height - 10)], outline=(0, 200, 200), width=2)
        draw.text((20, 20), msg, fill=(200, 200, 0), font=font, spacing=4)
        # Moving marker for visual change
        x = int((t * 40) % (self.width - 40)) + 20
        y = int((t * 25) % (self.height - 40)) + 20
        draw.ellipse([(x - 10, y - 10), (x + 10, y + 10)], outline=(255, 100, 0), width=3)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=self.quality)
        return buf.getvalue()

    def capture_jpeg(self) -> bytes:
        # Thread-safe capture to avoid concurrent access
        with self._lock:
            if self._picam is None:
                return self._placeholder_frame()

            # Picamera2 returns an RGB numpy array with this configuration
            arr = self._picam.capture_array()
            img = Image.fromarray(arr)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=self.quality)
            return buf.getvalue()

    def mjpeg_generator(self) -> Generator[bytes, None, None]:
        # multipart/x-mixed-replace generator yielding JPEG parts
        delay = max(0.0, 1.0 / max(1, self.fps))
        while True:
            frame = self.capture_jpeg()
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
            time.sleep(delay)


# Singleton camera instance
camera = Camera()
