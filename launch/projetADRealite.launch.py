import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # 3. On déclare NOTRE propre nœud de vision
    vision_node = Node(
        package='projet',          # Le nom de votre package
        executable='vision_node',  # Le nom défini dans le setup.py (console_scripts)
        name='vision_node_subscriber',
        output='screen'            # Permet de voir les 'print' ou 'logger' dans le terminal
    )

    control_node = Node(
        package='projet',
        executable='control_node',
        name='control_node_publisher',
        output='screen'
    )

    # 4. On assemble le tout et on l'envoie à ROS 2
    ld = LaunchDescription()
    ld.add_action(vision_node)
    ld.add_action(control_node)

    return ld