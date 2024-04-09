from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    gz_world_name = LaunchConfiguration('gz_world_name')
    px4_ns = LaunchConfiguration('px4_ns')
    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='AbuDhabi'
    )
    px4_ns_arg = DeclareLaunchArgument(
        'px4_ns',
        default_value='px4_1'
    )  
    # param Bridge
    param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/AbuDhabi/set_pose@ros_gz_interfaces/srv/SetEntityPose']
    )
    
    spoofer_node = Node(
        package='gnss_meaconing_attack',
        executable='spoofer_traj',
        name="spoofer",
        output = 'screen',
        parameters=[
            {'px4_ns': px4_ns},
            {'gz_world_name': gz_world_name},
        ]    
    )

    return LaunchDescription([
        gz_world_name_arg,
        px4_ns_arg,
        param_bridge,
        spoofer_node,
    ])