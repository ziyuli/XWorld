from teaching_task import *
import cv2
import numpy as np
import random
import math

meta       = "/home/ziyuli/XWorld/xrobot/data/ModelCategoryMapping.csv";
gift       = "/home/ziyuli/XWorld/xrobot/data/gift/gift.json";
crate1     = "./crate_1/crate.urdf";
cat1       = "./cat_1/cat.urdf";

models     = [crate1, gift, cat1]
m_tags     = ["crate1", "gift", "cat"]
conf0      = {"pair" : [cat1, gift, -1, 1]}
conf1      = {"single" : [crate1, -1, 9]}

class XWorld3DNavTargetNear(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetNear, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.CreateArena()
		self.env.LoadTag(meta);

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[3,0,3], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.6,0.0])

		self.env.LoadModels(models, m_tags)
		self.env.SpawnModelsConf(conf0)
		self.env.ResolvePath()
		self.env.SpawnModelsConf(conf1)
		self.env.SpawnModels()
		self.env.Initialize()

		self.target = "cat"
		self.target_object = self.env.QueryObjectByLabel(self.target)[0]
		self.near_objects = self.env.QueryObjectNearObject(self.target_object, True, 4.5)
		print self.near_objects

		self._bind("S -> start")
		self._bind("G -> '" + self.target + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):

		reward, time_out = self._time_reward()
		next_stage = "navigation"

		super(XWorld3DNavTargetNear, self).display_rgb(self.sentence)

		if not time_out:
			object_at_center = self.env.QueryObjectAtCameraCenter()
			if object_at_center.GetLabel() != "Nothing" and \
			   object_at_center.GetLabel() != "Wall" and \
			   object_at_center.GetLabel() != "Floor":
				if object_at_center in self.near_objects:
					print "Find"
					reward = self._successful_goal(reward)
					next_stage = "navigation"
				else:
					print "Wrong"
					reward = self._failed_goal(reward)
					next_stage = "navigation"
		else:
			next_stage = "navigation"

		return [next_stage, reward, self.sentence]

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.idle
		stages["navigation"] = self.navigation
		stages["terminal"] = self.idle
		return stages

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(m_tags)
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I0 | I1 | I2 | I3 | I4
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I0 -> A NP G
		I1 -> A NP G 'please' '.'
		I2 -> 'Please' A NP G '.'
		I3 -> NP G 'is' 'your' D '.'
		I4 -> Y A NP G '?'
		A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to' | 'collect'
		NP -> 'the' 'object' N
		N -> 'near' | 'by' | 'besides'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"