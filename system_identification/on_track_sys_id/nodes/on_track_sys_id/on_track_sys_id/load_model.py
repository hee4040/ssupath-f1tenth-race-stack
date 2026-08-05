import yaml
import os
from .dotdict import DotDict
from ament_index_python.packages import get_package_share_directory

def get_dict(model_name):
    """ Load YAML parameters from a specified model file. """
    model, tire = model_name.split("_")

    # Get package path in ROS 2
    package_path = get_package_share_directory('on_track_sys_id')
    
    model_file_path = os.path.join(package_path, 'models', f'{model_name}.txt')
    # model_file_path = os.path.join(package_path, 'models', f'{model}.txt')

    # Load YAML file
    with open(model_file_path, 'r') as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name):
    """ Convert dictionary parameters into DotDict format. """
    param_dict = get_dict(model_name)
    params = DotDict(param_dict)
    return params
