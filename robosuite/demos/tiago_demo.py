import robosuite as suite
import os
from robosuite.devices import Keyboard
import numpy as np

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
    ignore_done=False,
    control_freq=20,
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

device = Keyboard(env)
env.viewer.add_keypress_callback(device.on_press)
env.reset()

while True:

    env.reset()
    env.render()
    device.start_control()
    
    while True:
        print("here")
        input_dict = device.input2action()
        if input_dict is None:
            break
        action_vector = robot.create_action_vector(input_dict)

        env.step(action_vector)
        env.render()