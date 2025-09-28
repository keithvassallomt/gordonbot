import importlib
import importlib.metadata
import json
import sys

PKG_NAME = "lgpio"

def main() -> None:
    info = {}

    # Metadata version (pip/apt registered)
    try:
        info["metadata_version"] = importlib.metadata.version(PKG_NAME)
    except importlib.metadata.PackageNotFoundError:
        info["metadata_version"] = None

    # Import the module to see what Python actually loads
    try:
        lgpio = importlib.import_module(PKG_NAME)
    except Exception as exc:
        info["import_error"] = repr(exc)
    else:
        info["module_file"] = getattr(lgpio, "__file__", None)
        info["module_version_attr"] = getattr(lgpio, "__version__", None)
        info["has_bias_consts"] = {
            name: hasattr(lgpio, name)
            for name in ("SET_BIAS_DISABLE", "SET_BIAS_PULL_UP", "SET_BIAS_PULL_DOWN")
        }

    json.dump(info, sys.stdout, indent=2)
    print()

if __name__ == "__main__":
    main()
