import robosuite as suite
import os

CONTROLER_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "default_tiago.json")
composite_controller = suite.load_composite_controller_config(CONTROLER_CONFIG_PATH)

env = suite.make(
    "PickPlace",
    robots=["Tiago"],
    gripper_types=["Robotiq140Gripper"],
    controller_configs=composite_controller,
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    render_camera=["agentview", "robot0_robotview"],
    renderer="mujoco",
)

env.reset()

robot = env.robots[0]
robot.print_action_info()

default_controller_config = robot.robot_model.default_controller_config
print("Default Controller Config:", default_controller_config)

action_dict = {
    "torso": [0.2],  # [z]
    "base": [-1.0, 0.0, 1.0], # [x, y, theta]
}

while True:
    action = robot.create_action_vector(action_dict)
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display