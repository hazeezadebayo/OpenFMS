# USAGE:
# Terminal 1:
# cd docker_ws/env/dev1
# export DISPLAY=:0.0
# xhost +local:docker
# docker-compose up --build

# Terminal 2:
# docker exec -it dev1_tailscaled_1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch viro_simple_fleet fleet_client.launch.py

# Terminal 3:
# docker exec -it dev1_tailscaled_1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; python3 viro_simple_fleet/scripts/fleet_mngr_main.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # prefix = "/home/hazeezadebayo/docker_ws/ros2_ws"
    prefix = "/app/ros2_ws"
    package_name = "viro_simple_fleet"
    ws_name = "nav2_ign"
    # os.path.join(get_package_share_directory('turtlebot3'),'maps','turtlebot3_world.yaml')

    return LaunchDescription([

        Node(
            package='viro_simple_fleet',
            executable='fleet_client_monitor.py',
            name='viro_simple_fleet',
            output='screen',
            parameters=[
                {'scan_topic': "/scan"}, # <!-- --> default - " "
                {'robot_description_topic': "/robot_description"},
                {'map_topic': "/map"},
                {'odom_topic': "/odom"},
                {'amcl_topic': "/amcl_pose"},
                {'base_frame_name': "base_link"}
            ]
        ),

        Node(
            package='viro_simple_fleet',
            executable='fleet_client_main.py',
            name='viro_simple_fleet',
            output='screen',
            parameters=[
                {'robot_id': 'TB3_2'}, # <!-- --> default - " 3 "
                {'fleet_id': "kullar"}, # <!-- --> default - " "
                # {'fleet_base_map_pgm': prefix+"/"+ws_name+"/src/"+package_name+"/maps/my_cartographer_map.pgm"},
                # {'fleet_base_map_yaml': prefix+"/"+ws_name+"/src/"+package_name+"/maps/my_cartographer_map.yaml"},
                {'fleet_base_map_pgm': os.path.join(get_package_share_directory('turtlebot3'),'maps','turtlebot3_world.pgm')},
                {'fleet_base_map_yaml': os.path.join(get_package_share_directory('turtlebot3'),'maps','turtlebot3_world.yaml')},
                {'fleet_floor1_map_pgm': os.path.join(get_package_share_directory(package_name),'maps','devshop.pgm')},
                {'fleet_floor1_map_yaml': os.path.join(get_package_share_directory(package_name),'maps','devshop.yaml')},
                {'fleet_coverage_plan_yaml': ""},
                {'hostname': "192.168.1.93"}, # home-istabul: ("192.168.1.91") computer network ip
                {'database': "postgres"}, # <!-- --> default - " "
                {'username': "postgres"},
                {'pwd': "root"},
                {'port': 5432},
                {'wheel_seperation': 0.1}, # <!-- --> default - " "
                {'wheel_radius': 0.04},
                {'angular_vel': 0.9},
                {'linear_vel': 0.4},
                {'db_readwrite_freq': 5.0}, # 6.0
                {'current_pose': ['-1.82', '-0.54', '0.0']} # ['-1.82', '0.202', '0.0'] --> "['x', 'y', 'th']"
            ]
        ),

    ])
