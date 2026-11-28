from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import yaml


def nodes(context):
    """读取这次启动传进来的参数，决定要启动哪些节点。"""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    source, model, uart = value('source'), value('model'), value('uart')
    url, mode = value('camera_url'), value('mode')
    config = value('config')
    with open(config, encoding='utf-8') as stream:
        settings = yaml.safe_load(stream) or {}
    yolo = settings.get('inference_node', {}).get('ros__parameters', {}).get('yolo', {})
    model = model or yolo.get('model_path', '')
    trans = settings.get('task_node', {}).get('ros__parameters', {}).get('trans', {})
    # 预览上画的参考点跟当前模式保持一致：靠近测试用 approach_test_node，其余用任务节点。
    point_section = 'approach_test_node' if mode == 'approach' else 'task_node'
    point = settings.get(point_section, {}).get('ros__parameters', {}).get('point', {})
    point_x, point_y = float(point.get('x0', 640.0)), float(point.get('y0', 550.0))
    camera = settings.get('camera_node', {}).get('ros__parameters', {})
    simulate_uart = value('simulate_uart') == 'true'
    if simulate_uart and (not uart.startswith('/dev/pts/') or mode != 'approach'):
        raise ValueError('用 tools/run.sh 为靠近测试启动 STM32 模拟器')
    fps = int(value('camera_fps') or camera.get('FPS', 60))
    FOURCC = value('camera_fourcc') or camera.get('FOURCC', 'MJPG')
    if source not in ('usb', 'http') or (source == 'http' and not url):
        raise ValueError('source: usb / http（需要 camera_url）')
    if mode not in ('preview', 'approach', 'camera_test'):
        raise ValueError('mode: preview / approach / camera_test')
    if mode == 'camera_test' and (model or uart):
        raise ValueError('摄像头测试只运行采图和预览')
    if uart and not model:
        raise ValueError('串口模式需提供 YOLO 模型和 USB / HTTP 摄像头')

    # 先读 robot.yaml，再用本次 launch 传进来的参数覆盖同名配置。
    def node(package, executable, **overrides):
        return Node(package=package, executable=executable, output='screen',
                    parameters=[config, overrides])

    # 默认链路：相机 → 感知节点转发画面 → HTTP 服务。
    preview = {}
    for name in ('preview_width', 'preview_height'):
        if value(name):
            preview[name] = int(value(name))
    actions = [
        node('desktop_server_pkg', 'desktop_server_node', camera_source=source, mode=mode, yolo_enabled=bool(model), port=int(value('port')), **{'point.x0': point_x, 'point.y0': point_y, **preview}),
        node('vision_pkg', 'perception_node', detect=bool(model), pair_safety=bool(model),
             camera_settle_ms=trans.get('camera_settle_ms', 400)),
        node('vision_pkg', 'camera_node', device=value('camera_device'),
             url=url, FPS=fps, FOURCC=FOURCC),
    ]
    if model or value('safety_detector') == 'true':
        actions.append(node('vision_pkg', 'safety_detector_node', paired=bool(model)))
    # 有模型才做推理。串口由 uart 启用，靠近测试和正式任务只选其中一个。
    if model:
        actions.append(node('vision_pkg', 'inference_node', **{'yolo.model_path': model}))
    if uart:
        actions.append(node('communication_pkg', 'serial_node', team=value('team'), max_pitch_deg=camera.get('max_pitch_deg', 80), **{'uart.device': uart}))
    if mode == 'approach':
        overrides = {'drive': bool(uart) and not simulate_uart, 'publish_motion': simulate_uart}
        if value('color'):
            overrides['color'] = value('color')
        actions.append(node('tests_pkg', 'approach_test_node', **overrides))
    elif mode == 'camera_test':
        actions.append(node('tests_pkg', 'camera_test_node'))
    elif uart:
        task = {'vision_timeout_s': float(value('vision_timeout_s')),
                'grab.width': int(camera.get('width', 1280)), 'grab.height': int(camera.get('height', 720))}
        if value('sim_distribute') == 'true':
            task.update({'distribute.use_sim_clock': True, 'delivery.enabled': value('safety_detector') == 'true',
                         'distribute.move_ms': 6200,
                         'distribute.scatter_ms': 1800, 'distribute.scatter_forward': -100,
                         'distribute.base_turn': 170,
                         'grab.camera_deg': 50, 'grab.near_y': 650})
        actions.append(node('action_pkg', 'task_node', **task))
    return actions


def generate_launch_description():
    config = Path(get_package_share_directory('robot_launch')) / 'config/robot.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('source', default_value='usb'),
        DeclareLaunchArgument('camera_url', default_value=''),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('camera_fps', default_value=''),
        DeclareLaunchArgument('camera_fourcc', default_value=''),
        DeclareLaunchArgument('port', default_value='18080'),
        DeclareLaunchArgument('preview_width', default_value=''),
        DeclareLaunchArgument('preview_height', default_value=''),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('uart', default_value=''),
        DeclareLaunchArgument('team', default_value=''),
        DeclareLaunchArgument('vision_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('sim_distribute', default_value='false'),
        DeclareLaunchArgument('simulate_uart', default_value='false'),
        DeclareLaunchArgument('mode', default_value='preview'),
        DeclareLaunchArgument('safety_detector', default_value='false'),
        DeclareLaunchArgument('color', default_value=''),
        OpaqueFunction(function=nodes),
    ])
