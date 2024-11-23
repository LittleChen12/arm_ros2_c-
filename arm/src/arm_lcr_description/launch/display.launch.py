from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    RegisterEventHandler, 
    TimerAction, 
    EmitEvent,
    Shutdown,
    ExecuteProcess,
    LogInfo  # 直接从launch.actions导入
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('arm_lcr_description')
    default_model_path = urdf_tutorial_path / 'urdf/arm_lcr_description.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    #每个joint的单独控件
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 添加你的控制器节点
    # 使用简单的 joint_state_publisher，仅设置初始位置
    controller_active_arg = DeclareLaunchArgument(
        'controller_active', 
        default_value='false',
        choices=['true', 'false']
    )
    initial_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'rate': 10,
            'publish_default_positions': True,
            'default_positions': {
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                'joint6': 0.0
            }
        }],
        # 只在控制器未激活时启动
        condition=UnlessCondition(LaunchConfiguration('controller_active'))
    )

    # # publish controller_node,but delay 2 seconds
    # delayed_controller_node = TimerAction(
    #     period=2.0,  # 延迟2秒启动
    #     actions=[Node(
    #         package='llc_arm',
    #         executable='robot_controller',
    #         name='arm_controller',
    #         output='screen',
    #         emulate_tty=True,  # 启用完整的终端输出
    #         parameters=[{
    #         'controller_active': True  # 设置控制器激活标志
    #         }]
    #     )]
    # )
    # # 添加你的控制器节点
    # controller_node = Node(
    #     package='llc_arm',  # 改为你的控制器包名
    #     executable='robot_controller',  # 改为你的控制器节点可执行文件名
    #     name='arm_controller',
    #     output='screen',
    #     emulate_tty=True,  # 启用完整的终端输出
    #     # 如果需要设置参数，可以添加：
    #     parameters=[{
    #         'robot_description': robot_description,
    #         # 其他参数
    #         'param1': 'value1',
    #         'param2': 'value2'
    #     }]
    # )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        # controller_node
    ])
