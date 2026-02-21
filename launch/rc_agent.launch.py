import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    enable_gscam2 = LaunchConfiguration('enable_gscam2').perform(context).lower() in (
        'true', '1', 'yes'
    )
    enable_gst_pipeline = LaunchConfiguration('enable_gst_pipeline').perform(context).lower() in (
        'true', '1', 'yes'
    )

    rc_agent_share = get_package_share_directory('rc_agent')
    gscam_params = os.path.join(rc_agent_share, 'params', 'gscam2.yaml')
    gst_pipeline_params = os.path.join(rc_agent_share, 'params', 'gst_pipeline.yaml')
    basecontrol_params = os.path.join(rc_agent_share, 'params', 'basecontrol.yaml')
    colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    gst_bridge_path = ''
    if colcon_prefix:
        candidate = os.path.join(colcon_prefix, 'gst_bridge', 'lib', 'gst_bridge')
        if os.path.isdir(candidate):
            gst_bridge_path = candidate

    actions = []
    gst_pipeline_available = False
    gscam2_available = False
    if enable_gscam2:
        try:
            get_package_share_directory('gscam2')
            gscam2_available = True
        except PackageNotFoundError:
            actions.append(
                LogInfo(
                    msg='gscam2 package not found; rc_agent will run without video streaming.'
                )
            )

    if enable_gst_pipeline:
        try:
            get_package_share_directory('gst_pipeline')
            gst_pipeline_available = True
        except PackageNotFoundError:
            actions.append(
                LogInfo(
                    msg='gst_pipeline package not found; no fallback video streaming available.'
                )
            )

    selected_params = None
    if gscam2_available and gst_pipeline_available:
        actions.append(
            LogInfo(
                msg='gscam2 and gst_pipeline available; using gscam2 for streaming.'
            )
        )
    elif gscam2_available:
        actions.append(
            LogInfo(
                msg='gscam2 available; gst_pipeline not available or disabled.'
            )
        )
    elif gst_pipeline_available:
        actions.append(
            LogInfo(
                msg='gscam2 unavailable; using gst_pipeline for streaming.'
            )
        )
    else:
        actions.append(
            LogInfo(
                msg='No streaming backend available; rc_agent will run without video streaming.'
            )
        )
    if gscam2_available:
        gscam_extra_params = {}
        gscam_extra_env = {}
        if gst_bridge_path:
            gscam_extra_params['gst_plugin_path'] = gst_bridge_path
            gscam_extra_env['GST_PLUGIN_PATH'] = gst_bridge_path
        actions.append(
            Node(
                package='gscam2',
                executable='gscam_main',
                namespace=namespace,
                output='screen',
                additional_env=gscam_extra_env,
                parameters=[
                    gscam_params,
                    {
                        'use_sim_time': use_sim_time,
                        **gscam_extra_params,
                    },
                ],
            )
        )
        selected_params = gscam_params
    elif gst_pipeline_available:
        actions.append(
            Node(
                package='gst_pipeline',
                executable='pipeline_node',
                name='gst_pipeline_node',
                namespace=namespace,
                output='screen',
                additional_env={'GST_PLUGIN_PATH': gst_bridge_path} if gst_bridge_path else {},
                parameters=[
                    gst_pipeline_params,
                    {'use_sim_time': use_sim_time},
                ],
            )
        )
        selected_params = gst_pipeline_params

    rc_agent_params = [basecontrol_params, {'use_sim_time': use_sim_time}]
    if selected_params:
        rc_agent_params.insert(0, selected_params)
    if not selected_params:
        rc_agent_params.append({'video_pipeline': ''})

    actions.append(
        Node(
            package='rc_agent',
            executable='rc_agent',
            namespace=namespace,
            output='screen',
            parameters=rc_agent_params,
        )
    )

    return actions


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation time for all nodes',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace',
        ),
        DeclareLaunchArgument(
            'enable_gscam2',
            default_value='true',
            choices=['true', 'false'],
            description='Launch gscam2 UDP bridge when available',
        ),
        DeclareLaunchArgument(
            'enable_gst_pipeline',
            default_value='true',
            choices=['true', 'false'],
            description='Launch gst_pipeline UDP bridge when available',
        ),
        OpaqueFunction(function=_launch_setup),
    ]

    return LaunchDescription(arguments)
