from isaacsim import SimulationApp

# 若不需要界面，把 headless 改成 True
simulation_app = SimulationApp({"headless": False})

# 新 API（地面与几何体）
from isaacsim.core.api.world import World
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.objects.cuboid import DynamicCuboid

# USD/Kit 接口（用于灯光与资产引用与位姿计算）
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf
import omni.usd
import omni.kit.app
import os

# ---------- 启用 ROS2 bridge 扩展（不启 humble.rclpy） ----------
os.environ.setdefault("ROS_DOMAIN_ID", "0")
ext_mgr = omni.kit.app.get_app().get_extension_manager()
ext_mgr.set_extension_enabled("isaacsim.ros2.bridge", True)

# ---------- 指定 Franka 的本地 USD 路径（请先下载或同步到本地/Nucleus） ----------
# 方案A：本地文件路径（推荐）
FRANKA_LOCAL_USD = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

# 方案B：Nucleus 路径（如果你把目录同步至 Nucleus）
# FRANKA_LOCAL_USD = "omniverse://localhost/NVIDIA/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/FrankaPanda.usd"

def reference_asset(prim_path: str, asset_url: str) -> bool:
    """把 asset_url 引用到 prim_path，并尝试加载其 payload"""
    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(Sdf.Path(prim_path), "Xform")
    try:
        prim.GetReferences().AddReference(asset_url)
        # 关键：加载该 prim 的内容
        try:
            prim.Load()
        except Exception:
            pass
        return True
    except Exception as e:
        print(f"[Wind Note] 引用资产失败：{asset_url} —— {e}")
        return False

# 打印子树与搜索工具
def list_subtree(root_path, max_count=60):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"[Wind Note] 子树根不存在：{root_path}")
        return []
    names = []
    for prim in Usd.PrimRange(root):
        names.append(prim.GetPath().pathString)
        if len(names) >= max_count:
            break
    print("[Wind Note] 子树预览（前 %d 条）：" % len(names))
    for p in names:
        print("  -", p)
    return names

def find_prim_by_name(root_path, target_name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == target_name:
            return prim.GetPath().pathString
    return None

# 计算某个 prim 的世界坐标（取平移向量）
def get_world_translation(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"[Wind Note] prim 不存在：{prim_path}")
        return None
    xformable = UsdGeom.Xformable(prim)
    m = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return Gf.Vec3d(t[0], t[1], t[2])

world = World(stage_units_in_meters=1.0)

# 地面与柔光天幕
ground = GroundPlane(prim_path="/World/ground", name="ground")
world.scene.add(ground)

stage = omni.usd.get_context().get_stage()
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.GetIntensityAttr().Set(2000.0)
dome.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))

# 引用 Franka：根级 /Franka（按你的布局）
franka_root_path = "/Franka"
ok = reference_asset(franka_root_path, FRANKA_LOCAL_USD)
if not ok:
    print("[Wind Note] 请确认 Franka USD 已下载至本地（或改用 omniverse:// 路径）。")

# 把 Franka 根稍微挪一下（可按需调整）
UsdGeom.XformCommonAPI(stage.GetPrimAtPath(franka_root_path)).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))

# 先创建方块，稍后对齐到 link2
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="my_cube",
    position=(0.4, 0.0, 0.5),  # 初始占位
    size=0.1,
)
world.scene.add(cube)

world.reset()

# 在资产落地后，扫描 /Franka 子树，定位 link2（命名可能不同，准备多个候选）
linked = False
frames = 0
while simulation_app.is_running():
    world.step(render=True)
    frames += 1

    if ok and not linked and frames in (5, 10, 20, 40, 80):
        list_subtree(franka_root_path, max_count=200)
        candidates = ["panda_link2", "link2", "panda_link_2", "Franka_link2", "panda2"]
        link2_path = None
        for name in candidates:
            link2_path = find_prim_by_name(franka_root_path, name)
            if link2_path:
                break
        if link2_path is None:
            print("[Wind Note] 暂未找到 link2（尝试名：panda_link2/link2/panda_link_2/Franka_link2/panda2），稍后再试。")
        else:
            print(f"[Wind Note] 找到 link2：{link2_path}")
            pos = get_world_translation(link2_path)
            if pos is not None:
                target = pos + Gf.Vec3d(0.3, 0.0, 0.0)
                cube_prim = stage.GetPrimAtPath("/World/Cube")
                UsdGeom.XformCommonAPI(cube_prim).SetTranslate(target)
                print(f"[Wind Note] 已将方块对齐到 link2 旁：{tuple(target)}")
                linked = True

simulation_app.close()