import os

def get_base_dir():
    """Return the base directory of the current script."""
    return os.path.dirname(os.path.abspath(__file__))

base_url = get_base_dir()