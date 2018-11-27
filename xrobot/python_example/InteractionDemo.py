import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *

oven       = "../data/oven/oven.json";
drawer     = "../data/drawer/drawer.json";
drawer2    = "../data/drawer2/drawer.json";
gift       = "../data/gift/gift.json";
laptop     = "../data/laptop/laptop.json";
cake       = "../data/cake/cake.json";
trashcan   = "../data/trashcan/trashcan.json";
screen     = "../data/screen/screen.json";
piano      = "../data/piano/piano.json";

class XRobot3DInteractions(XWorld3DTask):
    def __init__(self, env):
        super(XRobot3DInteractions, self).__init__(env)
        self.sentence = "Interaction Demo ."

    def get_stages(self):
        stages = dict()
        stages["idle"] = self.start
        stages["navigation"] = self.navigation
        return stages

    def start(self):
        self.reset()
        self.env.CreateArena()
        self.env.SpawnAnObject(oven, [4,0,2], [1,0,0,0], 1.0, "oven", True)
        self.env.SpawnAnObject(drawer, [6,0,2], [1,0,0,0], 1.0, "drawer", True)
        self.env.SpawnAnObject(gift, [8,0,2], [1,0,0,0], 1.0, "gift", True)
        self.env.SpawnAnObject(laptop, [10,0,2], [1,0,0,0], 1.0, "laptop", True)
        self.env.SpawnAnObject(cake, [4,0,6], [1,0,0,0], 1.0, "cake", True)
        self.env.SpawnAnObject(trashcan, [6,0,6], [1,0,0,0], 1.0, "trashcan", True)
        self.env.SpawnAnObject(screen, [8,0,6], [1,0,0,0], 1.0, "screen", True)
        self.env.SpawnAnObject(piano, [10,0,6], [1,0,0,0], 1.0, "piano", True)
        self.env.SpawnAnObject(drawer2, [4,0,10], [1,0,0,0], 1.0, "drawer2", True)

        self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
        self.env.AttachCameraTo(self.agent, [0.3,1.6,0.0])
        self.env.Initialize()

        return ["navigation", 0.0, self.sentence]

    def navigation(self):
        super(XRobot3DInteractions, self).display_rgb(self.sentence)
        return ["navigation", 0.0, self.sentence]


class XRobotEnv(object):
    def __init__(self):
        self.env = Playground(640, \
                              360, \
		                      HEADLESS, \
                              RENDER_QUALITY_NORMAL, \
                              1)

        self.task_group = TaskGroup("TaskGroup")
        self.task_group.add_task("Navigation_1", XRobot3DInteractions(self.env))

    def reset(self):
        self.env.Clear()

    def step(self, action):
        self.task_group.run_stage()
        self.env.UpdateSimulationWithAction(action)

    def render(self):
        self.env.UpdateRenderer()

    def preprocess_observation(self, ob):
        return ob.astype("float32")

    def observation_dims(self):
        return self.env.GetObservationSpace();

    def action_dims(self):
        return self.env.GetActionSpace();

    def game_over(self):
        return False


env = XRobotEnv()
env.reset();

start = time.time()
while (not env.game_over()):

    action = NO_ACTION # Do Nothing

    # action inputs from keyboard
    key = cv2.waitKey(1)
    if key == 119:   # W
        action = 0
    elif key == 97:  # A
        action = 2
    elif key == 115: # S
        action = 1
    elif key == 100: # D
        action = 3
    elif key == 49:  # 1 Pick
        action = 8
    elif key == 50:  # 2 Drop
        action = 9
    elif key == 48:  # kp9 Up
        action = 4
    elif key == 57: # kp0 Down
        action = 5
    elif key == 54: # kp6 Open
        action = 12
    elif key == 51: # kp3 Close
        action = 13
    elif key == 55: # kp7 Enable Interact
        action = 11
    elif key == 56: # kp8 Disable Interact
        action = 13
    elif key == 27: # ESC
        break

    # update
    env.step(action)
    env.render()