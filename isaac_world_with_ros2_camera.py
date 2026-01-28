from isaacsim import SimulationApp

# ========== 启动 Isaac ==========
simulation_app = SimulationApp({"headless": False})

import isaacsim.ros2.bridge as ros2_bridge
print(dir(ros2_bridge))