from teaching_task import *
import cv2
import numpy as np
import random

meta       = "../data/ModelCategoryMapping.csv";
gift       = "../data/gift/gift.json";
crate1     = "../data/crate_1/crate.urdf";

models     = [crate1, gift]
m_tags     = ["crate1", "gift"]
conf       = {"single" : [crate1, -1, 4, gift, -1, 5]}

class XWorld3DNavTargetAvoid(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetAvoid, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.CreateArena()
		self.env.EnableInventory(4)
		self.env.LoadTag(meta);

		x = random.randint(4, 14)
		y = random.randint(4, 14)

		self.env.SpawnAnObject(crate1, [x,0,y], [1,0,0,0], 1.0, "crate1", True)
		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[3,0,3], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.6,0.0])
		self.env.ResolvePath()

		self.env.LoadModels(models, m_tags)
		self.env.SpawnModelsConf(conf)
		self.env.SpawnModels()
		self.env.Initialize()

		self._bind("S -> start")
		self._bind("G -> '" + "gift" + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):
		
		reward, time_out = self._time_reward()
		next_stage = "navigation"

		super(XWorld3DNavTargetAvoid, self).display_rgb(self.sentence)

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter("crate1"):
				reward = self._successful_goal(reward)
				next_stage = "terminal"
			elif self.env.QueryObjectWithLabelAtCameraCenter("gift"):
				reward = self._failed_goal(reward)
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
		all_goal_names = self._get_all_goal_names_as_rhs(["gift"])
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I0 | I1 | I2 | I4 | I5 | I6
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I0 -> V G '.'
		I1 -> V G 'please' '.'
		I2 -> 'Please' V G '.'
		I4 -> E G 'is' 'your' D '.'
		I5 -> E G 'is' 'the' D '.'
		I6 -> Y VV G '?'
		V -> 'do' 'not' A | 'avoid'
		VV -> 'not' A | 'avoid'
		E -> 'anything' 'except' | 'anything' 'but'
		A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to' | 'collect'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"
