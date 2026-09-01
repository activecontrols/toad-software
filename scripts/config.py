"""Configuration loader for serial port and other settings."""
import json
import os

# Default configuration
DEFAULT_CONFIG = {
    "serial": {
        "port": "/dev/ttyUSB0",
        "baudrate": 57600
    }
}


def load_config(config_path: str = "config.json") -> dict:
    """Load configuration from JSON file.
    
    Args:
        config_path: Path to config.json file. Defaults to "config.json" in current directory.
    
    Returns:
        Configuration dictionary with serial port settings.
    """
    if os.path.exists(config_path):
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            # Merge with defaults to ensure all keys exist
            return {**DEFAULT_CONFIG, **config}
        except (json.JSONDecodeError, IOError) as e:
            print(f"Warning: Failed to load config from {config_path}: {e}")
            print("Using default configuration")
            return DEFAULT_CONFIG
    else:
        print(f"Note: Config file not found at {config_path}, using defaults")
        return DEFAULT_CONFIG


def get_serial_config(config_path: str = "config.json") -> tuple[str, int]:
    """Get serial port configuration.
    
    Args:
        config_path: Path to config.json file.
    
    Returns:
        Tuple of (port, baudrate)
    """
    config = load_config(config_path)
    return config["serial"]["port"], config["serial"]["baudrate"]
