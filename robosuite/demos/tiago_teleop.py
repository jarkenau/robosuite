import os
import time
import numpy as np
from copy import deepcopy

import robosuite as suite
from robosuite.devices import Keyboard
from robosuite.controllers import load_composite_controller_config
from robosuite.controllers.composite.composite_controller import WholeBody
from robosuite.wrappers import DataCollectionWrapper

if __name__ == "__main__":

    MAX_FPS = 30
    CONTROLLER_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "tiago_controller_config.json")

    controller_config = load_composite_controller_config(
        controller=CONTROLLER_CONFIG_PATH,
    )

    env = suite.make(
        env_name="Lift",
        robots=["Tiago"],
        controller_configs=controller_config,
        has_renderer=True,
        renderer="mujoco",
        has_offscreen_renderer=False,
        render_camera=["agentview", "robot0_robotview"],
        ignore_done=True,
        use_camera_obs=False,
        reward_shaping=True,
        control_freq=20,
    )

    # NOTE: This has to stay, otherwise the base won't move https://github.com/ARISE-Initiative/robosuite/issues/704 
    tmp_directory = "/tmp/{}".format(str(time.time()).replace(".", "_"))
    env = DataCollectionWrapper(env, tmp_directory)

    device = Keyboard(
        env=env,
        pos_sensitivity=1.0,
        # TODO: find a better way to scale the rotation speed
        rot_sensitivity=4.0
    )

    while True:
        env.reset()
        env.render()

        task_completion_hold_count = -1 
        device.start_control()

        # Keep track of prev gripper actions when using since they are position-based and must be maintained when arms switched
        all_prev_gripper_actions = [
            {
                f"{robot_arm}_gripper": np.repeat([0], robot.gripper[robot_arm].dof)
                for robot_arm in robot.arms
                if robot.gripper[robot_arm].dof > 0
            }
            for robot in env.robots
        ]

        while True:
            start = time.time()
            active_robot = env.robots[device.active_robot]
            
            input_ac_dict = device.input2action()
            
            if input_ac_dict is None:
                break

            action_dict = deepcopy(input_ac_dict)  # {}
            
            # only use incremental control for the arms
            for arm in active_robot.arms:
                action_dict[arm] = input_ac_dict[f"{arm}_delta"]

            # Maintain gripper state for each robot but only update the active robot with action
            env_action = [robot.create_action_vector(all_prev_gripper_actions[i]) for i, robot in enumerate(env.robots)]
            env_action[device.active_robot] = active_robot.create_action_vector(action_dict)
            env_action = np.concatenate(env_action)
            for gripper_ac in all_prev_gripper_actions[device.active_robot]:
                all_prev_gripper_actions[device.active_robot][gripper_ac] = action_dict[gripper_ac]

            env.step(env_action)
            env.render()

            if task_completion_hold_count == 0:
                break

            # state machine to check for having a success for 10 consecutive timesteps
            if env._check_success():
                if task_completion_hold_count > 0:
                    task_completion_hold_count -= 1  # latched state, decrement count
                else:
                    task_completion_hold_count = 10  # reset count on first success timestep
            else:
                task_completion_hold_count = -1  # null the counter if there's no success

            if MAX_FPS is not None:
                elapsed = time.time() - start
                diff = 1 / MAX_FPS - elapsed
                if diff > 0:
                    time.sleep(diff)

        env.close()
