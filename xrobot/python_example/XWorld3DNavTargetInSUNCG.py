from teaching_task import *
import cv2
import numpy as np
import random
import os
import os.path

suncg_dir    = "../data/suncg";
suncg_meta   = suncg_dir + "/ModelCategoryMapping.csv";
suncg_house0 = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

goals        = ["door", "trash_can", "refrigerator", "chair", "fruit_bowl", "dishwasher", "microwave", "coffee_machine", "kettle"]

class XWorld3DNavTargetInSUNCG(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetInSUNCG, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.CreateSceneFromSUNCG()
		self.env.LoadSUNCG(suncg_house0, suncg_meta, suncg_dir)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[-6,0.1,-1], [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		self.target = goals[random.randint(0, len(goals) - 1)]

		self._bind("S -> start")
		self._bind("G -> '" + self.target + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):
		
		reward, time_out = self._time_reward()
		next_stage = "navigation"

		super(XWorld3DNavTargetInSUNCG, self).display_rgb(self.sentence)

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter(self.target):
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
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
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
		A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to' | 'collect'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"