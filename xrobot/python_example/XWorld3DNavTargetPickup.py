from teaching_task import *
import cv2
import numpy as np
import random

meta       = "/home/ziyuli/XWorld/xrobot/data/ModelCategoryMapping.csv";
gift       = "/home/ziyuli/XWorld/xrobot/data/gift/gift.json";
crate1     = "./crate_1/crate.urdf";
crate03    = "./crate_0.3/crate.urdf";

models     = [crate1, crate03]
m_tags     = ["large crate", "small crate"]
conf0       = {"single" : [crate03, -1, 5]}
conf1       = {"single" : [crate1, -1, 5]}

class XWorld3DNavTargetPickup(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetPickup, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.CreateArena()
		self.env.EnableInventory(4)
		self.env.LoadTag(meta);

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[3,0,3], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.6,0.0])
		self.env.LoadModels(models, m_tags)
		self.env.SpawnModelsConf(conf0)
		self.env.SpawnModelsConf(conf1)
		self.env.SpawnModels()

		self.env.Initialize()

		self._bind("S -> start")
		self._bind("G0 -> '" + "small' 'crate" + "'")
		self._bind("G1 -> '" + "large' 'crate" + "'")
		self.sentence = self._generate()

		return ["pickup", 0.0, self.sentence]

	def pickup(self):
		
		print "[stage] pickup"

		reward, time_out = self._time_reward()
		next_stage = "pickup"

		super(XWorld3DNavTargetPickup, self).display_rgb(self.sentence)

		if not time_out:
			desired_event = set(["Grasp", "small crate"])
			if set(self.env.QueryLastEvent()) & desired_event == desired_event:
				next_stage = "dropdown"


		return [next_stage, reward, self.sentence]

	def dropdown(self):

		print "[stage] dropdown"
		
		reward, time_out = self._time_reward()
		next_stage = "dropdown"

		super(XWorld3DNavTargetPickup, self).display_rgb(self.sentence)

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter("small crate"):
				small_crate = self.env.QueryObjectAtCameraCenter()
				if small_crate.GetPosition()[1] > 0.5:
					reward = self._successful_goal(reward)
					next_stage = "terminal"
				else:
					reward = self._failed_goal(reward)
					next_stage = "terminal"
			else:
				e = set(self.env.QueryLastEvent()) & set(["PutDown", "Nothing"])
				if e == set(["PutDown", "Nothing"]):
					next_stage = "dropdown"
					[next_stage, reward, self.sentence]
				elif e:
					next_stage = "pickup"
		else:
			next_stage = "terminal"

		return [next_stage, reward, self.sentence]

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.idle
		stages["pickup"] = self.pickup
		stages["dropdown"] = self.dropdown
		stages["terminal"] = self.idle
		return stages

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(["small' 'crate", "large' 'crate"])
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I1 | I2 | I3 | I6 
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I1 -> A0 G0 'and' A1 G1 'please' '.'
		I2 -> 'Please' A0 G0 'and' A1 G1 '.'
		I3 -> A0 G0 'and' A1 G1 '.'
		I6 -> Y A0 G0 'and' A1 G1 '?'
		A1 -> 'put' 'onto' | 'drop' 'onto'
		A0 -> 'pick' 'up' | 'grasp' | 'take' | 'collect'
		Y  -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		G0 --> %s
		G1 --> %s
		""" % (all_goal_names, all_goal_names)
		return grammar_str, "S"