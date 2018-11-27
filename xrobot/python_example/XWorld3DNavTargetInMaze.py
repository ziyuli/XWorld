from teaching_task import *
import cv2
import numpy as np
import random

door1      = "../data/door_green/door.json";
door2      = "../data/door_red/door.json";
door3      = "../data/door_yellow/door.json";
door4      = "../data/door_blue/door.json";
door5      = "../data/door_purple/door.json";

key2       = "../data/key_red/key.urdf";
key3       = "../data/key_yellow/key.urdf";
key4       = "../data/key_blue/key.urdf";
key5       = "../data/key_purple/key.urdf";

door       = "../data/door/door.urdf";
wall       = "../data/wall0/floor.urdf";
floor_0    = "../data/floor0/floor.urdf";
floor_1    = "../data/floor1/floor.urdf";
floor_2    = "../data/floor2/floor.urdf";
crate1     = "../data/crate_1/crate.urdf";
crate03    = "../data/crate_0.3/crate.urdf";
cat1       = "../data/cat_1/cat.urdf";

floor_test  = "../data/floor/floor.urdf";
wall_test   = "../data/wall/floor.urdf";

models     = [crate1]
m_tags     = ["crate1"]
conf       = {"single" : [crate1, -1, 20]}

class XWorld3DNavTargetInMaze(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetInMaze, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.EnableInventory(4)
		self.env.CreateRandomGenerateScene()
		self.env.SetLighting({ 
			"ssr" : True, \
			"exposure" : 0.0
		})

		doors  = [door2, door3, door4, door5]
		keys   = [key2, key3, key4, key5]
		d_tags = ["red", "yellow", "blue", "purple"]
		models = [crate1, crate03, cat1]
		m_tags = ["large crate", "small crate", "cat"]

		self.env.MakeObjectPickable("small crate")
		self.env.LoadBasicObjects(doors, keys, d_tags, door1, wall_test, [floor_test])
		start = self.env.LoadSceneConfigure(5, 5, 3, 2)
		lastgroup = self.env.GetRoomGroups()[-1]

		conf   = {"single" : [cat1, lastgroup, 3], \
         		  "stack"  : [crate03, crate1, 1, 1, 1]}

		self.env.LoadModels(models, m_tags)
		self.env.SpawnModelsConf(conf)
		self.env.SpawnModels()

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			start, [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		self._bind("S -> start")
		self._bind("G -> '" + "cat" + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):
		
		reward, time_out = self._time_reward()
		next_stage = "navigation"

		super(XWorld3DNavTargetInMaze, self).display_rgb(self.sentence)

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter("cat"):
				reward = self._successful_goal(reward)
				next_stage = "terminal"
		else:
			next_stage = "terminal"

		return [next_stage, reward, self.sentence]

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.idle
		stages["navigation"] = self.navigation
		stages["terminal"] = self.idle
		return stages

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(["cat"])
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I0 | I1 | I2 | I3 | I4 | I5 | I6
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I0 -> G
		I1 -> A G 'please' '.'
		I2 -> 'Please' A G '.'
		I3 -> A G '.'
		I4 -> G 'is' 'your' D '.'
		I5 -> G 'is' 'the' D '.'
		I6 -> Y A G '?'
		A -> 'find' | 'navigate' 'to' | 'reach' | 'move' 'to' | 'collect'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"