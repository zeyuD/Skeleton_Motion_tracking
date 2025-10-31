import json
import socket
import os

def load_machine_config(config_path=None):
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'machine_config.json')

    hostname = socket.gethostname().lower()

    with open(config_path, 'r') as f:
        configs = json.load(f)

    for name, conf in configs.items():
        if name.lower() in hostname:
            config = conf.copy()

            # Auto-fill based on OS or environment
            if 'wsl' in hostname or 'microsoft' in os.uname().release.lower():
                config['code_dir'] = config.get('wsl_code_dir')
                config['data_dir'] = config.get('wsl_data_dir')
            elif os.name == 'nt':
                config['code_dir'] = config.get('win_code_dir')
                config['data_dir'] = config.get('win_data_dir')
            else:
                config['code_dir'] = config.get('code_dir')
                config['data_dir'] = config.get('data_dir')

            return config

    raise KeyError(f"No matching config found for hostname: {hostname}")


config = load_machine_config()
print("Code Directory:", config["code_dir"])
print("Data Directory:", config["data_dir"])