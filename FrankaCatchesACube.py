# ============================================================
# Isaac Sim 5.1 | Franka Panda: 先张开 -> 俯探 -> 抓取
# ============================================================

import os
import numpy as np

# 屏蔽环境警告
os.environ["OMNI_KIT_TESTS_DISABLE"] = "1"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ------------------------------------------------------------
# 基础 imports
# ------------------------------------------------------------
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.objects import DynamicCuboid

# ------------------------------------------------------------
# 1. World & Ground
# ------------------------------------------------------------
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# ------------------------------------------------------------
# 2. Cube（目标物体）
# ------------------------------------------------------------
# Cube 的中心位置。Cube 高度 0.05，放在 Z=0.025 刚好贴地
cube_initial_pos = np.array([0.5, 0.0, 0.025])
cube_size = 0.05

cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=cube_initial_pos,
    size=cube_size
)
world.scene.add(cube)

# ------------------------------------------------------------
# 3. Franka Panda（保留您的加载逻辑）
# ------------------------------------------------------------
stage = world.stage
FRANKA_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)

franka_root = stage.DefinePrim("/World/FrankaRoot", "Xform")
franka_prim = stage.DefinePrim("/World/FrankaRoot/Franka", "Xform")
franka_prim.GetReferences().AddReference(FRANKA_USD)
franka_prim.Load()

# 包装为机器人对象
franka_robot = world.scene.add(
    Franka(
        prim_path="/World/FrankaRoot/Franka", 
        name="my_franka"
    )
)

# ------------------------------------------------------------
# 4. 控制器初始化
# ------------------------------------------------------------
world.reset()

# 使用 5.1 正确的初始化方式：直接传入 gripper 对象
controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka_robot.gripper,
    robot_articulation=franka_robot
)

# ------------------------------------------------------------
# 5. 主循环逻辑
# ------------------------------------------------------------
task_started = False
warmup_steps = 0  # 用于“先张开”的计时器

print(">>> 场景准备就绪，请点击 Play 开始...")

while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        # --- 初始化阶段 ---
        if not task_started:
            print(">>> 任务开始：重置场景...")
            world.reset()
            cube.set_world_pose(position=cube_initial_pos, orientation=np.array([1,0,0,0]))
            controller.reset()
            task_started = True
            warmup_steps = 0 # 重置计时器

        # --- 步骤1: 强制先张开夹爪 (持续 60 帧) ---
        if warmup_steps < 60:
            # 获取“张开”的动作指令
            # action="open" 会施加力让夹爪打开
            open_action = franka_robot.gripper.forward(action="open")
            franka_robot.apply_action(open_action)
            
            if warmup_steps == 0:
                print(">>> 步骤1: 张开夹爪...")
            
            warmup_steps += 1
            continue  # 跳过下面的代码，直到 warmup 完成

        # --- 步骤2: 执行自动抓取流程 ---
        # 控制器会自动处理：移动到上方(Approach) -> 垂直下降(Descend) -> 闭合(Grasp) -> 提升(Lift)
        
        current_cube_pos, _ = cube.get_world_pose()
        
        # 这里的 picking_position 是抓取点
        # Cube中心在 0.025，我们稍微往下一丁点(0.02)或者正中心，确保手指能包住物体
        # 控制器内部逻辑会先移动到 target + offset (上方)，然后慢慢降落
        actions = controller.forward(
            picking_position=current_cube_pos,
            placing_position=current_cube_pos + np.array([0, 0, 0.2]),
            current_joint_positions=franka_robot.get_joint_positions()
        )

        if actions is not None:
            franka_robot.apply_action(actions)
        
        if controller.is_done():
            # 任务完成，保持最后姿态
            pass

    elif world.is_stopped():
        task_started = False
        warmup_steps = 0

simulation_app.close()