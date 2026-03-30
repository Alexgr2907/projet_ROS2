import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. On trouve le chemin vers le launch file de projet_2025
    projet2025_dir = get_package_share_directory('projet2025')
    gazebo_launch_file = os.path.join(projet2025_dir, 'launch', 'projet.launch.py') # Vérifiez si c'est bien gazebo.launch.py ou world.launch.py selon l'autocomplétion que vous aviez testée.

    # 2. On crée l'action d'inclure ce launch file (Cela lance Gazebo, le robot, les obstacles, etc.)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

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
    ld.add_action(gazebo_launch)
    ld.add_action(vision_node)
    ld.add_action(control_node)

    return ld