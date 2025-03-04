from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import xacro


def launch_setup(context):

    xacro_path = LaunchConfiguration('description_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')


    print(use_sim_time)



    joint_state_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_gui' : True,
                'rate' : 100,
                }],
            )

    xacro_content = xacro.process_file(xacro_path).toxml()
    
    robot_state_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_tf_static' : True,
                'publish_frequency' : 100.0,
                'robot_description' : xacro_content,
                'use_sim_time' : use_sim_time
                }],
            )    

    return [joint_state_node, robot_state_node]



def generate_launch_description():


    description_file_arg = DeclareLaunchArgument(
        name = 'description_file',
        default_value=os.path.join(
                get_package_share_directory('unitree_go2_description'), 
                'urdf', 
                'go2.urdf'),
    )

  

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false'
    )


    load_func = OpaqueFunction(function = launch_setup)


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d' + os.path.join(get_package_share_directory('unitree_go2_description'), 'config' ,'standalone.rviz')]
            )




    ld = LaunchDescription(
    [
        
        description_file_arg,
        use_sim_time_arg,
        load_func,
        rviz_node
    ]
    )


    return ld
