import robosuite as suite
from robosuite.devices import Keyboard
from copy import deepcopy

env = suite.make(
    "PickPlace",
    robots=["Tiago"],
    gripper_types=["Robotiq140Gripper"],
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
        input_ac_dict = device.input2action()
        action_dict = deepcopy(input_ac_dict)
        if input_ac_dict is None:
            break
        
        # set 
        for arm in robot.arms:
            action_dict[arm] = input_ac_dict[f"{arm}_delta"]
        
        action_vector = robot.create_action_vector(action_dict)

        env.step(action_vector)
        env.render()