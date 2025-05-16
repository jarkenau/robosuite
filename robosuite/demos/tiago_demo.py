import numpy as np
import robosuite as suite

env = suite.make(
    "PickPlace",
    robots=["Tiago"],
    gripper_types=["Robotiq140Gripper"],
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    render_camera=["agentview", "robot0_robotview"],
    renderer="mujoco",
)

env.reset()

while True:
    action = np.zeros(*env.action_spec[0].shape)
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display