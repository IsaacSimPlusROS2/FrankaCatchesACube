from isaacsim import SimulationApp

# ========== 启动 Isaac ==========
simulation_app = SimulationApp({"headless": False})

import omni.kit.app
from isaacsim.core.utils.extensions import enable_extension

# 先启用扩展（避免 Graph 在未初始化时崩溃）
enable_extension("omni.graph.action")
enable_extension("omni.syntheticdata")
enable_extension("isaacsim.ros2.bridge")  # 新版扩展名
enable_extension("omni.isaac.ros2_bridge")  # 兼容旧版
enable_extension("omni.replicator.core")    # 用于 render_product

# 让扩展完全加载
for _ in range(10):
    omni.kit.app.get_app().update()

# ========== ROS2 Bridge ==========
from pxr import UsdGeom
import omni.graph.core as og
import omni.replicator.core as rep
from pxr import UsdLux
from isaacsim.core.api import World
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.api.objects.cuboid import DynamicCuboid

# ---------- 创建世界 ----------
world = World(stage_units_in_meters=1.0)
stage = world.stage

# ---------- 加载 Franka ----------
FRANKA_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)

root = stage.DefinePrim("/World/FrankaRoot", "Xform")
UsdGeom.XformCommonAPI(root).SetTranslate((0, 0, 0))

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
camera_root = create_prim("/World/Franka/CameraRoot", "Xform")
camera_path = "/World/Franka/CameraRoot/Camera"
camera_prim = create_prim(
    prim_path=camera_path,
    prim_type="Camera"
)

xform_api = UsdGeom.XformCommonAPI(camera_root)
xform_api.SetTranslate((0.6, 0.0, 1.2))
xform_api.SetRotate((-60.0, 0.0, 0.0))

# ---------- 创建 Render Product（Replicator） ----------
render_product = rep.create.render_product(camera_path, resolution=(640, 480))

# 不同版本返回对象不同，优先取 .path
render_product_path = getattr(render_product, "path", None)
if render_product_path is None:
    render_product_path = str(render_product)

print("render_product_path =", render_product_path)
print("RenderProduct valid:", stage.GetPrimAtPath(render_product_path).IsValid())

# ---------- ROS2 图像发布（OmniGraph 方式） ----------
image_topic = "/franka/camera/image_raw"

graph_path = "/ROS2CameraGraph"
og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "camera_helper.inputs:execIn"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # 注意：这里必须是字符串 token，不要 Sdf.Path
            ("camera_helper.inputs:renderProductPath", render_product_path),
            ("camera_helper.inputs:topicName", image_topic),
            ("camera_helper.inputs:frameId", "franka_camera"),
        ],
    },
)

# ---------- 运行 ----------
world.reset()
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()