# ============================================================
# Isaac Sim 5.1 | Franka Panda: 搬运物体到B，随后机械臂移动到C
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
cube_initial_pos = np.array([0.5, 0.0, 0.025])
cube_size = 0.05

cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=cube_initial_pos,
    size=cube_size,
    color=np.array([1, 0, 0])
)
world.scene.add(cube)

# ------------------------------------------------------------
# 3. Franka Panda 加载逻辑
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

controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka_robot.gripper,
    robot_articulation=franka_robot
)

# ------------------------------------------------------------
# 5. 主循环逻辑
# ------------------------------------------------------------
task_started = False
warmup_steps = 0 
task_phase = 0 
# 0: 搬运 A -> B (物体随动)
# 1: 移动 B -> C (空手移动)
# 2: 完成

# 坐标定义
target_place_pos = np.array([0.0, 0.5, 0.05]) # B点：物体放置处
empty_move_pos   = np.array([0.5, 0.0, 0.5])  # C点：机械臂最终去向 (高处)

print(">>> 场景准备就绪，点击 Play 开始...")

while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        # --- 初始化阶段 ---
        if not task_started:
            print(">>> 任务开始：重置...")
            world.reset()
            cube.set_world_pose(position=cube_initial_pos)
            controller.reset()
            task_started = True
            warmup_steps = 0 
            task_phase = 0

        # ====================================================
        # 预备: 张开夹爪
        # ====================================================
        if warmup_steps < 60:
            franka_robot.apply_action(franka_robot.gripper.forward(action="open"))
            warmup_steps += 1
            continue 

        # ====================================================
        # 阶段 0: 搬运物体 A -> B
        # ====================================================
        if task_phase == 0:
            current_cube_pos, _ = cube.get_world_pose()
            
            # 这是一个完整的抓取-放置动作
            actions = controller.forward(
                picking_position=current_cube_pos,
                placing_position=target_place_pos,
                current_joint_positions=franka_robot.get_joint_positions()
            )

            if actions is not None:
                franka_robot.apply_action(actions)
            
            if controller.is_done():
                print(f"✅ 搬运完成，物体留在 B 点。")
                controller.reset() # 必须重置以进行下一次移动
                task_phase = 1 

        # ====================================================
        # 阶段 1: 机械臂移动 B -> C (不带物体)
        # ====================================================
        elif task_phase == 1:
            # 核心逻辑：
            # 我们再次使用 controller，但将 picking_position 和 placing_position 
            # 都设为目标 C 点。这会欺骗控制器让它直接飞过去。
            # 或者更简单：设定 pick 为当前位置，place 为目标位置，但因为物体已经不在那了，
            # 机械臂只是在空中执行一套“假装抓取并放下”的动作。
            
            # 最干净的方法：让控制器执行一个从“当前位置”到“C点”的任务
            actions = controller.forward(
                picking_position=empty_move_pos, # 直接去C点抓
                placing_position=empty_move_pos, # 就在C点放
                current_joint_positions=franka_robot.get_joint_positions()
            )
            
            if actions is not None:
                franka_robot.apply_action(actions)

            if controller.is_done():
                print(f"✅ 机械臂已到达 C 点。")
                task_phase = 2

        # ====================================================
        # 阶段 2: 停在 C 点
        # ====================================================
        elif task_phase == 2:
            pass

    elif world.is_stopped():
        task_started = False
        warmup_steps = 0
        task_phase = 0

simulation_app.close()