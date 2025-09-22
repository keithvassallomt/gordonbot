import os
import time
import shutil
import subprocess
import psutil
from typing import Dict, Any


def _get_throttling() -> Dict[str, Any] | None:
    """
    Read Raspberry Pi throttling/undervoltage flags via `vcgencmd get_throttled` if available.
    Returns a dict of decoded flags or None if unavailable.
    """
    vc = shutil.which("vcgencmd")
    if not vc:
        return None
    try:
        out = subprocess.run([vc, "get_throttled"], capture_output=True, text=True, check=True)
        # Example output: "throttled=0x50005"
        text = out.stdout.strip()
        if "=" in text:
            text = text.split("=", 1)[1]
        value = int(text, 16)
        # Bits per Raspberry Pi docs
        flags = {
            "raw": value,
            "under_voltage_now": bool(value & (1 << 0)),
            "freq_capped_now": bool(value & (1 << 1)),
            "throttled_now": bool(value & (1 << 2)),
            "soft_temp_limit_now": bool(value & (1 << 3)),
            "under_voltage_occured": bool(value & (1 << 16)),
            "freq_capped_occured": bool(value & (1 << 17)),
            "throttled_occured": bool(value & (1 << 18)),
            "soft_temp_limit_occured": bool(value & (1 << 19)),
        }
        return flags
    except Exception:
        return None


def get_cpu_temperature() -> float | None:
    try:
        temps = getattr(psutil, "sensors_temperatures", lambda: None)()
        if not temps:
            return None
        if "cpu_thermal" in temps and temps["cpu_thermal"]:
            return temps["cpu_thermal"][0].current
        if "coretemp" in temps and temps["coretemp"]:
            return temps["coretemp"][0].current
        if "rp1_adc" in temps and temps["rp1_adc"]:
            return temps["rp1_adc"][0].current
        first = next(iter(temps.values()))
        if first:
            return first[0].current
    except Exception:
        pass
    return None


def get_system_diagnostics() -> Dict[str, Any]:
    """
    Return system diagnostics including CPU, load, temperatures, memory, disk, uptime, and Pi throttling flags.

    Includes:
    - cpu_util_percent: instantaneous utilization percentage (psutil.cpu_percent)
    - load_average: 1/5/15 minute load averages (process queue lengths)
    - load_per_cpu: load averages normalized by logical CPU count (useful heuristic)
    - cpu_temperature: SoC/CPU temperature in °C if available
    - rp1_temperature: RP1 I/O controller temperature in °C if available (Pi 5)
    - memory: total/available/used/percent
    - swap: total/free/used/percent (if present)
    - disk_root: filesystem usage for '/'
    - uptime_seconds: seconds since boot
    - throttling: Raspberry Pi get_throttled flags (if vcgencmd available)

    Notes:
    * `cpu_util_percent` (e.g. 3.0 %) is not the same as load average (e.g. 0.09).
      Load average is the average runnable + uninterruptible tasks over a time window.
    """
    diag: Dict[str, Any] = {}

    # Instantaneous CPU utilization percentage (across all cores)
    diag["cpu_util_percent"] = psutil.cpu_percent(interval=0.3)

    # Load averages (Unix only). Prefer psutil.getloadavg() if present; fallback to os.getloadavg().
    try:
        if hasattr(psutil, "getloadavg"):
            load1, load5, load15 = psutil.getloadavg()  # type: ignore[attr-defined]
        else:
            load1, load5, load15 = os.getloadavg()
        diag["load_average"] = {"1m": load1, "5m": load5, "15m": load15}
        cpus = psutil.cpu_count(logical=True) or 1
        diag["load_per_cpu"] = {"1m": load1 / cpus, "5m": load5 / cpus, "15m": load15 / cpus}
    except (OSError, AttributeError):
        diag["load_average"] = None
        diag["load_per_cpu"] = None

    # Temperatures: expose both CPU and RP1 when available
    cpu_temp = get_cpu_temperature()
    rp1_temp = None
    try:
        temps = getattr(psutil, "sensors_temperatures", lambda: None)()
        if temps:
            if "rp1_adc" in temps and temps["rp1_adc"]:
                rp1_temp = temps["rp1_adc"][0].current
    except Exception:
        if cpu_temp is None:
            cpu_temp = None
    diag["cpu_temperature"] = cpu_temp
    diag["rp1_temperature"] = rp1_temp

    # Memory
    try:
        vm = psutil.virtual_memory()
        diag["memory"] = {
            "total": vm.total,
            "available": vm.available,
            "used": vm.used,
            "percent": vm.percent,
        }
        sm = psutil.swap_memory()
        diag["swap"] = {
            "total": sm.total,
            "free": sm.free,
            "used": sm.used,
            "percent": sm.percent,
        }
    except Exception:
        diag["memory"] = None
        diag["swap"] = None

    # Disk (root filesystem)
    try:
        du = psutil.disk_usage("/")
        diag["disk_root"] = {
            "total": du.total,
            "used": du.used,
            "free": du.free,
            "percent": du.percent,
        }
    except Exception:
        diag["disk_root"] = None

    # Uptime
    try:
        boot = psutil.boot_time()
        diag["uptime_seconds"] = max(0, int(time.time() - boot))
        diag["boot_time"] = int(boot)
    except Exception:
        diag["uptime_seconds"] = None
        diag["boot_time"] = None

    # Raspberry Pi throttling flags (optional)
    diag["throttling"] = _get_throttling()

    return diag
