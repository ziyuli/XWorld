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
brick03    = "../data/brick_0.3/brick.urdf";
cakeknife  = "../data/cake_knife/cake_knife.urdf";
box_red    = "../data/box/red.urdf";
box_green  = "../data/box/green.urdf";
box_blue   = "../data/box/blue.urdf";
box_yellow = "../data/box/yellow.urdf";
box_cyan   = "../data/box/cyan.urdf";
box_purple = "../data/box/purple.urdf";
box_orange = "../data/box/orange.urdf";
box_white  = "../data/box/white.urdf";
goals = ["oven", "drawer", "gift", "laptop", "cake", "trashcan", "screen", "piano", "box", "cakeknife"]

class XRobot3DInteractions(XWorld3DTask):
	def __init__(self, env):
		super(XRobot3DInteractions, self).__init__(env)
		self.prev_find = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["navigation"] = self.navigation
		return stages

	def start(self):
		self.reset()
		self.env.EnableInventory(1)
		self.env.CreateArena(4, 4)
		self.env.MakeObjectPickable("box")
		self.env.MakeObjectPickable("cakeknife")
		self.env.SpawnAnObject(oven, [4,0,2], [1,0,0,0], 1.0, "oven", True)
		self.env.SpawnAnObject(drawer, [6,0,2], [1,0,0,0], 1.0, "drawer", True)
		self.env.SpawnAnObject(gift, [8,0,2], [1,0,0,0], 1.0, "gift", True)
		self.env.SpawnAnObject(laptop, [10,0,2], [1,0,0,0], 1.0, "laptop", True)
		self.env.SpawnAnObject(cake, [4,0,6], [1,0,0,0], 1.0, "cake", True)
		self.env.SpawnAnObject(trashcan, [6,0,6], [1,0,0,0], 1.0, "trashcan", True)
		self.env.SpawnAnObject(screen, [8,0,6], [1,0,0,0], 1.0, "screen", True)
		self.env.SpawnAnObject(piano, [11,0,6], [1,0,0,0], 1.0, "piano", True)
		self.env.SpawnAnObject(drawer2, [4,0,10], [1,0,0,0], 1.0, "drawer", True)
		self.env.SpawnAnObject(box_red, [8,0.15,10], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_green, [9,0.15,10], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_blue, [10,0.15,10], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_yellow, [8,0.15,11], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_cyan, [9,0.15,11], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_purple, [10,0.15,11], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_orange, [11,0.15,10], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(box_white, [11,0.15,11], [1,0,0,0], 1.0, "box", False)
		self.env.SpawnAnObject(cakeknife, [4, 0.9, 10], [1,0,0,0], 1.0, "cakeknife", False)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0,1.5,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(True)

		self._bind("S -> nothing")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):

		find = self.env.QueryObjectAtCameraCenter().GetLabel()
		
		if self.prev_find != find:
			if find in goals:
				self._bind("S -> find")
				self._bind("G -> '" + find + "'")
			else:
				self._bind("S -> nothing")
			self.sentence = self._generate()

		self.prev_find = find

		super(XRobot3DInteractions, self).display_rgb(self.sentence, 640, 480)
		return ["navigation", 0.0, self.sentence]

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> find | nothing
		find -> I0 | I1
		nothing -> I2
		I0 -> 'you' 'find' G | 'this' 'is' G
		I1 -> G
		I2 -> 'nothing' | 'not' 'sure' | 'unknown' 'object'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"


class XRobotEnv(object):
	def __init__(self):
		self.env = Playground(640, \
							  480, \
							  DEBUG_VISUALIZATION, \
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
	elif key == 49:  # kp1 Pick
		action = 8
	elif key == 50:  # kp2 Drop
		action = 9
	elif key == 52:  # kp4 Attach
		action = 6
	elif key == 53:  # kp5 Detach
		action = 7
	elif key == 48:  # kp9 Up
		action = 4
	elif key == 57:  # kp0 Down
		action = 5
	elif key == 54:  # kp6 Open
		action = 12
	elif key == 51:  # kp3 Close
		action = 13
	elif key == 55:  # kp7 Enable Interact
		action = 11
	elif key == 56:  # kp8 Disable Interact
		action = 13
	elif key == 27:  # ESC
		break

	# update
	env.step(action)
	env.render()