from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_config():
    return (
        MoveItConfigsBuilder(
            robot_name="ava",
            package_name="ava_moveit_config",
        )
        .to_moveit_configs()
    )
