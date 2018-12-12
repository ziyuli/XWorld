import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *
from XBridge import *

goals = ["box"]

class XRobot3DInteractions(XWorld3DTask):
	def __init__(self, env):
		super(XRobot3DInteractions, self).__init__(env)

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["task"] = self.task
		return stages

	def start(self):
		self.reset()
		self.env.EnableInventory(4)
		self.env.CreateArena(3, 3)
		self.env.LoadXWorldScene("../data/scene.xworld")

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [0,0,0], \
			[-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0,1.6,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(True)
		self.env.DisplayInventory(False)

		self._bind("S -> task")
		self.sentence = self._generate()

		return ["task", 0.0, self.sentence]

	def task(self):

		super(XRobot3DInteractions, self).display_rgb(self.sentence, 640, 480)

		return ["task", 0.0, self.sentence]


	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> task | correct | wrong 
		task -> I0
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		I0 -> 'Demo'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"


class XWorld3D(object):
	def __init__(self):
		self.env = Playground(640, \
							  480, \
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

	def game_over(self):
		return False

# XWorld3D
env = XWorld3D()
env.reset()

# XWorld3D - Unity Bridge
server = XWorldServer('', 1236, env)
server.start()

while (not env.game_over()):

	action = NO_ACTION 

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
	elif key == 27:  # ESC
		break

	# update
	env.step(action)
	env.render()

server.stop()