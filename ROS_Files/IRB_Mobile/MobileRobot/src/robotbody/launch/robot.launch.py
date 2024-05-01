import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    launch_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_motion'),
                'launch/map.launch.py'))
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    x_arg = DeclareLaunchArgument('x',
    default_value= "1.0"
    )
    y_arg = DeclareLaunchArgument('y',
    default_value= "2.0"
    )
    yaw_arg = DeclareLaunchArgument('yaw',
    default_value= "0.0"
    )
    robot_description_dir = get_package_share_directory("robotbody")
    robot_urdf_path = os.path.join(robot_description_dir, "urdf", "mobile_manipulator.urdf.xacro")
    robot_sdf_path = os.path.join(robot_description_dir, "urdf", "mobile_manipulator.sdf")
    world_path = os.path.join(robot_description_dir, "worlds", "irb120.world")
    print("Sdf path = ", robot_sdf_path)
    print("URDF path = ", robot_urdf_path)
    
    #Remove later
    doc = xacro.parse(open(robot_urdf_path))
    robot_description_config_test = doc.toxml()
    robot_description_test = {'robot_description': robot_description_config_test}
    

    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments= {"verbose": "false", "world": world_path}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': ParameterValue(Command(['xacro ', robot_urdf_path]), value_type = str)} ],
    )
    
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", robot_sdf_path, "-entity", "mobile_manipulator", "-x", x, "-y", y, "-z", "0.2", "-Y", yaw],
        output="screen",
    )
    
    arm_position_controller_spawner= Node(
        package = 'controller_manager',
        executable = 'spawner',
        output = 'screen',
        arguments = ["arm_position_controller","--controller-manager","/controller_manager"]) 

    arm_velocity_controller_spawner= Node(
        package = 'controller_manager',
        executable = 'spawner',
        output = 'screen',
        arguments = ["arm_velocity_controller","--controller-manager","/controller_manager", "--inactive"])    
    
    joint_state_broadcaster_spawner= Node(
    package = 'controller_manager',
    executable = 'spawner',
    output = 'screen',
    arguments = ["joint_state_broadcaster","--controller-manager","/controller_manager"])
    
    
    global_planner_spawner = Node(
    package = 'robot_motion',
    executable = 'globalPlanner',
    output = 'screen',
    )
    
    

    local_planner_spawner = Node(
    package = 'robot_motion',
    executable = 'localPlanner',
    output = 'screen',
    )
    
    arm_controller_spawner = Node(
    package = 'robot_motion',
    executable = 'armController',
    output = 'screen',
    )
    
    grasp_object_spawner = Node(
    package = 'ros2_grasping',
    executable = 'attacher_action.py',
    output = 'screen',
    )
    
    
    

    delay_position_controller_spawner = RegisterEventHandler(event_handler = OnProcessExit(target_action = joint_state_broadcaster_spawner, on_exit = [arm_position_controller_spawner]))
    delay_velocity_controller_spawner = RegisterEventHandler(event_handler = OnProcessExit(target_action = arm_position_controller_spawner, on_exit = [arm_velocity_controller_spawner]))
    delay_broadcaster_spawner = RegisterEventHandler(event_handler = OnProcessExit(target_action = spawn_robot, on_exit = [joint_state_broadcaster_spawner]))
    delay_global_planner_spawner = RegisterEventHandler(event_handler = OnProcessExit(target_action = arm_velocity_controller_spawner, on_exit = [global_planner_spawner]))
    delay_local_planner_spawner = RegisterEventHandler(event_handler = OnProcessExit(target_action = global_planner_spawner, on_exit = [local_planner_spawner]))
    delay_arm_controller_spawner = RegisterEventHandler(event_handler = OnProcessExit(target_action = local_planner_spawner, on_exit = [arm_controller_spawner]))
    return LaunchDescription([
        launch_map,
        use_sim_time_arg,      
        x_arg,
        y_arg,
        yaw_arg, 
        robot_state_publisher_node,
        spawn_gazebo, 
        spawn_robot,
        delay_broadcaster_spawner,
        delay_position_controller_spawner,
        delay_velocity_controller_spawner,
        global_planner_spawner,
        local_planner_spawner,
        arm_controller_spawner,
        grasp_object_spawner,
        ])
