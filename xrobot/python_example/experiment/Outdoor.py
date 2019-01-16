from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random
import os
import os.path

suncg_dir    = "../../data/suncg"
suncg_meta   = suncg_dir + "/ModelCategoryMapping.csv"
suncg_house0 = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json"

terrain_textures      = ["../../data/terrain/gravel.jpg", "../../data/terrain/road.jpg"]
terrain_texture_scale = [5.0, 20.0]

goals = ["door", "trash_can", "refrigerator", "chair", "fruit_bowl", \
		 "dishwasher", "microwave", "coffee_machine", "kettle"]

class XWorld3DNavTargetInSUNCG(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetInSUNCG, self).__init__(env)
		self.count = 0

	def idle(self):
		self.reset()
		self.env.CreateSceneFromSUNCG()
		self.env.CreateTerrain(terrain_textures, terrain_texture_scale)
		self.env.PaintPuddle([10,10], 5.0, 0.1, 1)
		self.env.LoadTerrain([-1,1,1], [0.1,0.1])
		self.env.LoadSUNCG(suncg_house0, suncg_meta, suncg_dir, [10, 0, 10], 1)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[5,0.2,5], [-1,0,0,1.57], 1.0, "Agent", False)
		self.env.AttachCameraTo(self.agent, [0.0,1.3,0.0])
		self.env.Initialize()

		return ["navigation", 0.0, ""]

	def navigation(self):
		return ["navigation", 0.0, ""]

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.idle
		stages["navigation"] = self.navigation
		stages["terminal"] = self.idle
		return stages

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, 480, VISUALIZATION, NORMAL_NO_SHADOW, GPU0)
		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetSUNCG", XWorld3DNavTargetInSUNCG(self.env))
		self.first = True
		self.env.SetLighting({ "exposure" : 1.2 })

	def reset(self):
		self.env.Clear()

	def step(self, joints):

		if self.first != True:
			self.env.ControlJointVelocities(self.env.GetAgent(), joints, 50000)
			self.env.UpdateSimulation()

		self.first = False
		self.task_group.run_stage()

		self.env.UpdateRenderer()
		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( 480, 640, 4 )
		image_rgbd = cv2.flip(image_rgbd, 0)
		image_rgbd_resize = cv2.resize(image_rgbd, None, fx=0.8, fy=0.8)
		image_rgb = np.array(image_rgbd_resize[:,:,:3])
		image_d   = np.array(image_rgbd_resize[:,:,3:4])

		cv2.putText(image_rgb, "", (30,30), \
			cv2.FONT_HERSHEY_PLAIN, 1.25, (15,255,15), 1, cv2.LINE_AA);
		cv2.imshow("RGB", image_rgb)

	def render(self):
		self.env.UpdateRenderer()

	def game_over(self):
		return False

def main():
	env = XWorld3DEnv()
	vel_0, vel_1, vel_2, vel_3 = 0, 0, 0, 0

	while (not env.game_over()):

		vel_0, vel_1, vel_2, vel_3 = 0, 0, 0, 0

		# action inputs from keyboard
		key = cv2.waitKey(1)
		if key == 119:   # W
			vel_0 += 17.5
			vel_1 += 17.5
			vel_2 += 17.5
			vel_3 += 17.5
		elif key == 115:  # S
			vel_0 -= 17.5
			vel_1 -= 17.5
			vel_2 -= 17.5
			vel_3 -= 17.5
		elif key == 97: # A
			vel_0 -= 17.5
			vel_1 += 17.5
			vel_2 -= 17.5
			vel_3 += 17.5
		elif key == 100: # D
			vel_0 += 17.5
			vel_1 -= 17.5
			vel_2 += 17.5
			vel_3 -= 17.5
		elif key == 27:  # ESC
			break

		joints = { 2 : vel_0, 3 : vel_1, 4 : vel_2, 5 : vel_3 }
		env.step(joints)

if __name__ == '__main__':
	main()