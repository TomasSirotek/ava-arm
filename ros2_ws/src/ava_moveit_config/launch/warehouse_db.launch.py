from moveit_configs_utils.launches import generate_warehouse_db_launch
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ava", package_name="ava_moveit_config"
        ).to_moveit_configs()
    )
    return generate_warehouse_db_launch(moveit_config)
