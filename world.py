from isaacsim import SimulationApp
import os
os.environ["OMNI_KIT_TESTS_DISABLE"] = "1"

# ========== 启动 Isaac ==========
simulation_app = SimulationApp({
    "headless": False,
    "disable_rendering": False,
    "renderer": "RayTracedLighting"
})

import omni.kit.app
from isaacsim.core.utils.extensions import enable_extension

# 先启用扩展（避免 Graph 在未初始化时崩溃）
enable_extension("omni.graph.action")
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.replicator.core")

# 让扩展完全加载
for _ in range(10):
    omni.kit.app.get_app().update()

# ========== ROS2 Bridge ==========
import numpy as np
from pxr import UsdGeom
import omni.graph.core as og
import omni.replicator.core as rep
from pxr import UsdLux
from isaacsim.core.api import World
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.api.objects.cuboid import DynamicCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

# ---------- 创建世界 ----------
world = World(stage_units_in_meters=1.0)
stage = world.stage

# ---------- 加载 Franka ----------
FRANKA_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)

franka_root = stage.DefinePrim("/World/FrankaRoot", "Xform")
franka = stage.DefinePrim("/World/FrankaRoot/Franka", "Xform")
franka.GetReferences().AddReference(FRANKA_USD)
franka.Load()

# ---------- 方块 ----------
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=(0.6, 0.0, 0.4),
    size=0.08,
)
world.scene.add(cube)
cube_prim = stage.GetPrimAtPath("/World/Cube")
UsdGeom.Gprim(cube_prim).CreateDisplayColorAttr([(1.0, 0.2, 0.2)])

# -------- 灯光 ------------
sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
sun.CreateIntensityAttr(2000.0)
sun.CreateAngleAttr(1.0)

# ---------- 添加地面 ----------
create_prim("/World/GroundPlane", "Xform")
world.scene.add_default_ground_plane()

# ---------- 设置摄像头 ----------
# camera_root = create_prim("/World/Franka/CameraRoot", "Xform")
# camera_path = "/World/Franka/CameraRoot/Camera"
# camera_prim = create_prim(
#     prim_path=camera_path,
#     prim_type="Camera"
# )

# xform_api = UsdGeom.XformCommonAPI(camera_root)
# xform_api.SetTranslate((0.6, 0.0, 1.2))
# xform_api.SetRotate((-60.0, 0.0, 0.0))

# ---------- 创建 Render Product（Replicator） ----------
# render_product = rep.create.render_product(camera_path, resolution=(640, 480))

# # 不同版本返回对象不同，优先取 .path
# render_product_path = getattr(render_product, "path", None)
# if render_product_path is None:
#     render_product_path = str(render_product)

# print("render_product_path =", render_product_path)
# print("RenderProduct valid:", stage.GetPrimAtPath(render_product_path).IsValid())

# ---------- ROS2 图像发布（OmniGraph 方式） ----------
# image_topic = "/franka/camera/image_raw"

# graph_path = "/ROS2CameraGraph"
# og.Controller.edit(
#     {"graph_path": graph_path, "evaluator_name": "execution"},
#     {
#         og.Controller.Keys.CREATE_NODES: [
#             ("tick", "omni.graph.action.OnPlaybackTick"),
#             ("camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
#         ],
#         og.Controller.Keys.CONNECT: [
#             ("tick.outputs:tick", "camera_helper.inputs:execIn"),
#         ],
#         og.Controller.Keys.SET_VALUES: [
#             ("camera_helper.inputs:renderProductPath", render_product_path),
#             ("camera_helper.inputs:topicName", image_topic),
#             ("camera_helper.inputs:frameId", "franka_camera"),
#         ],
#     },
# )

# ---------- Franka 控制 ----------
franka_articulation = Articulation(prim_path="/World/FrankaRoot/Franka", name="franka")
world.scene.add(franka_articulation)

arm_joint_names = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]
finger_joint_names = [
    "panda_finger_joint1",
    "panda_finger_joint2",
]

def build_action(arm_positions, finger_position):
    joint_positions = franka_articulation.get_joint_positions()
    if joint_positions is None:
        return None   # ⬅️ 关键：直接放弃这一帧

    joint_positions = np.array(joint_positions)

    dof_names = franka_articulation.dof_names
    dof_index = {name: i for i, name in enumerate(dof_names)}

    for name, value in zip(arm_joint_names, arm_positions):
        joint_positions[dof_index[name]] = value

    for name in finger_joint_names:
        joint_positions[dof_index[name]] = finger_position

    return ArticulationAction(
        joint_positions=joint_positions,
        joint_indices=np.arange(len(joint_positions))  # ⭐关键
    )


# 俯身、张开、夹住、抬起的关节目标
pose_bend_down = [0.0, -0.9, 0.0, -2.0, 0.0, 1.6, 0.8]
pose_lift_up = [0.0, -0.4, 0.0, -1.4, 0.0, 1.2, 0.8]
gripper_open = 0.04
gripper_close = 0.0

sequence = [
    {"name": "bend_down", "arm": pose_bend_down, "gripper": gripper_open, "frames": 140},
    {"name": "open_gripper", "arm": pose_bend_down, "gripper": gripper_open, "frames": 40},
    {"name": "grasp", "arm": pose_bend_down, "gripper": gripper_close, "frames": 80},
    {"name": "lift", "arm": pose_lift_up, "gripper": gripper_close, "frames": 140},
]

# ---------- 运行 ----------
world.reset()

# 2. 预先跑几帧，让 PhysX / ArticulationView 真正建立起来
# 2. 先跑一小段时间，让 Simulation View 完全创建
for _ in range(50):
    world.step(render=True)

# 3. 再初始化 articulation（有的版本里 initialize 需要在世界运行后调用更稳）
franka_articulation.initialize()

# 4. 再跑几帧，确保 controller/view 绑定完成
for _ in range(10):
    world.step(render=True)

print("Num DOF:", franka_articulation.num_dof)
print("DOF names:", franka_articulation.dof_names)
print("Joint positions init:", franka_articulation.get_joint_positions())

step_index = 0
step_frame = 0
current_action = None

warmup = True
warmup_steps = 20

while simulation_app.is_running():

    if warmup:
        # 让仿真多跑几步，直到关节读取稳定
        jp = franka_articulation.get_joint_positions()
        if jp is not None:
            warmup_steps -= 1
        if warmup_steps <= 0:
            warmup = False
        world.step(render=True)
        continue
     
    if step_index < len(sequence):
        step = sequence[step_index]
        if step_frame == 0:
            current_action = build_action(step["arm"], step["gripper"])

        if current_action is not None and current_action.joint_positions is not None:
            franka_articulation.apply_action(current_action)

        step_frame += 1
        if step_frame >= step["frames"]:
            step_index += 1
            step_frame = 0
    else:
        if current_action is not None:
            franka_articulation.apply_action(current_action)

    world.step(render=True)

simulation_app.close()