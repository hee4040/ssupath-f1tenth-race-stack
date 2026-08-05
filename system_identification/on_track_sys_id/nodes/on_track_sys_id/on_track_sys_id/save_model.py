import yaml
import os
import rclpy
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory

def save(model, overwrite_existing=True, verbose=False):

    package_path = get_package_share_directory('on_track_sys_id')
    
    file_path = os.path.join(package_path, "models",f"{model['model_name']}_{model['tire_model']}.txt")
    # file_path = os.path.join(package_path, "models",f"{model['model_name']}.txt")
    if os.path.isfile(file_path):
        if verbose:
            print("Model already exists")
        if overwrite_existing:
            if verbose:
                print("Overwriting...")
        else:
            if verbose:
                print("Not overwriting.")
            return 0

    try:
        model = model.to_dict()
    except AttributeError:
        model = model
        
    with open(file_path, "w") as f:
        get_logger('on_track_sys_id').info(f"MODEL IS SAVED TO: {file_path}")
        yaml.dump(model, f, default_flow_style=False)