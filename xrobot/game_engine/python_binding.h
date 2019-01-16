#ifndef PLAYGROUND_PY_H_
#define PLAYGROUND_PY_H_

#define EXPERIMENTAL

#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include <boost/python.hpp>

#include "render_engine/render.h"
#include "map_grid.h"
#include "suncg.h"
#include "terrain.h"
#include "lidar.h"
#include "task.h"
#include "navigation.h"

using namespace xrobot;

typedef render_engine::GLContext CTX;

void list2vec(const boost::python::list& ns, std::vector<float>& v) {
	int L = len(ns);
	v.resize(L);
	for (int i=0; i<L; ++i) {
		v[i] = boost::python::extract<float>(ns[i]);
	}
}

inline boost::python::tuple vec2tuple(const glm::vec3 v) {
	return boost::python::make_tuple(v.x, v.y, v.z);
}

inline boost::python::tuple vec2tuple(const glm::vec4 v) {
	return boost::python::make_tuple(v.x, v.y, v.z, v.w);
}

inline glm::vec3 list2vec3(const boost::python::list& ns) {
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 3);
	return glm::vec3(ns_v[0], ns_v[1], ns_v[2]);
}

inline glm::vec4 list2vec4(const boost::python::list& ns) {
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 4);
	return glm::vec4(ns_v[0], ns_v[1], ns_v[2], ns_v[3]);
}

inline glm::quat list2quat(const boost::python::list& ns) {
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 4);
	return glm::angleAxis(ns_v[3], glm::vec3(ns_v[0], ns_v[1], ns_v[2]));
}

// --------------------------------------------------------------------------------------------

struct Range {
	glm::vec3 min, max;

	Range() {
		min = glm::vec3(0);
		max = glm::vec3(0);
	}

	Range(boost::python::list min_py, boost::python::list max_py) {
		min = list2vec3(min_py);
		max = list2vec3(max_py);
	}

	// Two AABBs are equal
	bool __eq__(const Range& other) { 
		return min == other.min && max == other.max;
	}

	// Two AABBs are intersect
	bool __ne__(const Range& other) { 
		return glm::all(glm::lessThanEqual(min, other.max)) &&
			   glm::all(glm::greaterThanEqual(max, other.min));
	}

	// Contains
	bool __gt__(const Range& other) {
		return glm::all(glm::lessThan(min, other.min)) &&
			   glm::all(glm::greaterThan(max, other.max));
	}

	// Contains
	bool __lt__(const Range& other) {
		return glm::all(glm::lessThan(other.min, min)) &&
			   glm::all(glm::greaterThan(other.max, max));
	}

	// To String
	static std::string __str__(const Range& self) {
		return std::string( 
			"(" +
				std::to_string(self.min.x) + ", " + 
				std::to_string(self.min.y) + ", " +
				std::to_string(self.min.z) + ", " +
				std::to_string(self.max.x) + ", " +
				std::to_string(self.max.y) + ", " +
				std::to_string(self.max.z) +
			")"
		);
	}

	boost::python::tuple GetMin() const { return vec2tuple(min); }
	boost::python::tuple GetMax() const { return vec2tuple(max); }
};

// --------------------------------------------------------------------------------------------

class Thing {
public:
	Thing();

	// Returns current position in (x y z)
	boost::python::tuple GetPosition();

	// Return current orientation in quaternion (x y z w). 
	boost::python::tuple GetOrientation();

	// Return the label
	std::string GetLabel();

	long __hash__() { return (long) (uintptr_t) robot_.lock().get(); }
	bool __eq__(const Thing& other) { 
		return robot_.lock().get() == other.robot_.lock().get();
	}
	static std::string __str__(const Thing& self) { return self.label_; }

	std::weak_ptr<RobotBase> GetPtr() const { return robot_; }
	void SetPtr(std::weak_ptr<RobotBase> robot) { robot_ = robot; Sync(); }

private:
	bool Sync();
	std::string label_;
	boost::python::tuple position_;
	boost::python::tuple orientation_;
	std::weak_ptr<RobotBase> robot_;
};

// --------------------------------------------------------------------------------------------
// Experimental

class NavAgent {
public:
	explicit NavAgent(const int uid, const std::string& label);

	std::string GetLabel() const { return label_; }
	int GetUid() const { return uid_; }
	bool __eq__(const NavAgent& other) { return uid_ == other.uid_; }
	static std::string __str__(const NavAgent& self) { return self.label_; }

private:
	int uid_;
	std::string label_;
};

// --------------------------------------------------------------------------------------------
// Playground defines the scene and renderer. Used to control the robot,
// generate a scene, enable certain feature and query the object.
class Playground {
public:

	// Create a empty playground with basic rendering parameters.
	// Use VISUALIZATION to enable flyover camera.
	Playground(const int w, 
			   const int h,
			   const int headless = 0, 
			   const int quality = 0,
			   const int device = 0); 
	~Playground();

	// Initialization
	void Initialize();

	// Update
	void Update();
	void UpdateRenderer();
	boost::python::dict UpdateSimulation();
	boost::python::dict UpdateSimulationWithAction(const int action);

	// Reset
	void Clear();
	void ClearSpawnObjectsExceptAgent();

	// Lighting
	void SetLighting(const boost::python::dict lighting);

	// Camera
	boost::python::tuple GetCameraPosition() const;
	boost::python::tuple GetCameraFront() const;
	boost::python::tuple GetCameraRight() const;
	boost::python::tuple GetCameraUp() const;
	float GetCameraYaw() const;
	float GetNearClippingDistance();
	float GetFarClippingDistance();
	int GetWidth() const { return w_; }
	int GetHeight() const { return h_; }
	boost::python::object GetCameraRGBDRaw();

	// Free camera
	void FreeCamera(const boost::python::list position, const float yaw, const float pitch);
	void UpdateFreeCamera(const boost::python::list position, const float yaw, const float pitch);

	// Attachable camera
	void AttachCameraTo(Thing object, const boost::python::list offset_py);
	void UpdateAttachCamera(const float pitch);

	// Single ray lidar
	void EnableLidar(const int num_rays, const float max_distance);
	boost::python::list UpdateLidar(const boost::python::list front_py, 
								    const boost::python::list up_py,
								    const boost::python::list position_py);

	// Inventory
	void EnableInventory(const int max_capacity = 1);
	void ClearInventory();
	void MakeObjectPickable(const std::string& tag);

	// Querying labels
	void AssignTag(const std::string& path, const std::string& tag);
	void LoadTag(const std::string& path);

	// Control
	void HoldActions(const bool hold);
	void MoveForward(const float speed = 1);
	void MoveBackward(const float speed = 1);
	void TurnLeft(const float speed = 1);
	void TurnRight(const float speed = 1);
	void LookUp();
	void LookDown();
	std::string Grasp();
	std::string PutDown();
	void Attach();
	void Detach();
	std::string Rotate(const boost::python::list angle_py);
	void Teleport(Thing object, 
				  const boost::python::list position_py,
				  const boost::python::list orientation_py);
	void ControlJointPositions(const Thing& object, 
					   		   const boost::python::dict joint_positions,
					   		   const float max_force);
	void ControlJointVelocities(const Thing& object, 
					   		    const boost::python::dict joint_velocities,
					   		    const float max_force);

	// Interactions
	boost::python::list EnableInteraction();
	void DisableInteraction();
	bool TakeAction(const int action_id);
	boost::python::list OpenInventory();
	void CloseInventory();
	void Use(const int object_id);
	bool UseObject(const int inventory_id);

	// Event
	boost::python::list QueryLastEvent();

	// Query
	bool QueryContact(Thing& object);
	bool QueryObjectAABBIntersect(Thing& object_a, Thing& object_b);
	bool QueryObjectWithLabelAtCameraCenter(const std::string& label);
	bool QueryObjectWithLabelAtForward(const std::string& label);
	bool QueryObjectWithLabelNearMe(const std::string& label);
	Thing QueryObjectAtCameraCenter();
	boost::python::list QueryObjectByLabel(const std::string& label);
	boost::python::list QueryObjectNearObject(Thing& object, 
											  const bool exlude = true, 
											  const float dist = 2.0f);

	// HUD
	void HighlightCenter(const bool highlight);
	void DisplayInventory(const bool display);

	// Others
	Thing GetAgent() const { return agent_; }
	boost::python::dict GetStatus() const;
	boost::python::dict GetActionSpace();
	boost::python::dict GetObservationSpace();
	boost::python::list GetGoals() const;


	// --------------------------------------------------------------------------------------------
	// Generate scene

	// Generate a empty scene with checkerboard style floors and walls
	void CreateArena(const int width = 5, const int length = 5);

	// Generate a empty scene only for loading SUNCG houese
	void CreateSceneFromSUNCG();

	// Generate a random generated maze
	void CreateRandomGenerateScene();

	// Generate a empty scene
	void CreateEmptyScene(const float min_x = -5, 
						  const float max_x =  5,
                          const float min_z = -5, 
                          const float max_z =  5);

	// --------------------------------------------------------------------------------------------
	// The following methods only can be used in random generated maze or arena
	
	boost::python::list LocateObjectInGrid(Thing& object);
	boost::python::list LocatePositionInGrid(const float x, const float z);
	boost::python::list GetRoomVisitSequence();
	boost::python::list GetRoomGroups();
	boost::python::list GetSpaceNearPosition(const boost::python::list position,
										     const float radius);
	boost::python::list LoadSceneConfigure(const int w, 
										   const int l, 
										   const int n, 
										   const int d);
	void LoadBasicObjects(const boost::python::list doors,
						  const boost::python::list keys,
						  const boost::python::list key_tags,
						  const std::string unlocked_door,
						  const std::string& wall,
						  const boost::python::list tiles);
	void LoadModels(const boost::python::list models,
					const boost::python::list tags);
	void SpawnModelsConf(const boost::python::dict conf);
	void SpawnModels();
	void ResolvePath();

	// --------------------------------------------------------------------------------------------
	// The following methods only can be used in empty scene

	void LoadXWorldScene(const std::string& filename);
	boost::python::dict GetXWorldScene() const { return json_scene_; }

	// --------------------------------------------------------------------------------------------
	
	// Load suncg house
	void LoadSUNCG(const std::string& house,
				   const std::string& metadata,
				   const std::string& suncg_data_dir,
				   const boost::python::list offset = boost::python::list(),
				   const int filter = -1);

	// --------------------------------------------------------------------------------------------
	
	// Populate an object
	Thing SpawnAnObject(const std::string& file, 
					    const boost::python::list position_py,
					    const boost::python::list orentation_py,
					    const float scale,
					    const std::string& label,
					    const bool fixed = true,
					    const bool occupy = true);

	// Remove an object
	void RemoveAnObject(Thing& object);


	// --------------------------------------------------------------------------------------------
	// Experiental
	// - Navigation Mesh
	// - Terrain

	// Enable navigations to use path-finding for a object or robot
	//
	// The range of the baking area is defined by two 'vectors'. 
	// The minimum y must smaller than the ground, and the maximum y cannot
	// greater than the ceiling.
	//
	// Path-finding relies on grid map which is generated by a special
	// depth camera on top of the scene. The range of the baking area
	// needs to be passed into this member function.
	void EnableNavigation(const boost::python::list min_corner, 
						  const boost::python::list max_corner,
						  const bool kill_after_arrived);

	// Bake the grid map for path-finding.
	void BakeNavigationMesh();

	// Assign a threshould for determine ground when baking the grid map
	// 
	// The threshould is a log-based value between 0 to 1
	//
	// The surface threshould will be automatically calculated base on
	// the major depth samples in the grid map. However, you may need to
	// assign it by yourself in some scenario
	void AssignSurfaceLevel(const float level);

	// Assign a scale up value to dilate the grid map
	//
	// In some cases, a large size agent cannot pass a small gap. You need
	// dilate the grid map to fill the gaps
	//
	// A negative value for erode the grid map
	void AssignAgentRadius(const float radius);


	// Assign a target to an agent
	void AssignNavigationAgentTarget(const NavAgent& agent,
		                             const boost::python::list position);

	// Spawn an agent with path-finding feature
	NavAgent SpawnNavigationAgent(const std::string& path,
							      const std::string& label,
							      const boost::python::list position,
							      const boost::python::list orientation);


	// Generate a empty scene contains terrain
	void CreateTerrain(const boost::python::list layers,
					   const boost::python::list scales,
					   const float height = 1.0f,
					   const int terrain_size = 30,
					   const int grid_size = 128);
	void PaintPuddle(const boost::python::list position,
					 const float scale,
					 const float height,
					 const int id = 0);
	void PaintCircle(const boost::python::list position,
					 const float scale,
					 const float height,
					 const int id = 0);
	void PaintBox(const boost::python::list min_aabb,
				  const boost::python::list max_aabb,
				  const float height,
				  const int id = 0);
	void PaintCurve(const boost::python::list control_points,
					const float scale,
					const int prec = 32,
					const int id = 0);
	void LoadTerrain(const boost::python::list clamp,
					 const boost::python::list noise);

	// --------------------------------------------------------------------------------------------
	
private:
	int w_, h_;
	int iterations_;
	float camera_aspect_;
	float camera_pitch_;
	float camera_yaw_;
	
	bool hold_actions_;
	bool highlight_objects_;
	bool kill_after_arrived_;
	bool gameover_;
	bool interact_;
	bool inventory_opened_;
	Thing agent_;

	boost::python::dict json_scene_;
	boost::python::list current_actions_;
	boost::python::list current_objects_;
	boost::python::list current_event_;

	std::vector<Thing> objects_;
	std::shared_ptr<Map> scene_;
	std::shared_ptr<Suncg> suncg_;
	std::shared_ptr<Terrain> terrain_;
	std::shared_ptr<Inventory> inventory_;
	std::shared_ptr<Navigation> crowd_;
	std::shared_ptr<Lidar> lidar_;
	std::shared_ptr<render_engine::Render> renderer_;
	render_engine::GLContext * ctx_;
	render_engine::Camera * main_camera_;
};

// --------------------------------------------------------------------------------------------
// Python Binding

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateTerrain_member_overloads,
									   Playground::CreateTerrain, 2, 5);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PaintPuddle_member_overloads,
									   Playground::PaintPuddle, 3, 4);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PaintCircle_member_overloads,
									   Playground::PaintCircle, 3, 4);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PaintBox_member_overloads,
									   Playground::PaintBox, 3, 4);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PaintCurve_member_overloads,
									   Playground::PaintCurve, 2, 4);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(EnableInventory_member_overloads, 
									   Playground::EnableInventory, 0, 1);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateEmptyScene_member_overloads, 
									   Playground::CreateEmptyScene, 0, 4);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LoadSUNCG_member_overloads, 
									   Playground::LoadSUNCG, 3, 5);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SpawnAnObject_member_overloads, 
									   Playground::SpawnAnObject, 5, 7);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MoveForward_member_overloads, 
									   Playground::MoveForward, 0, 1);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MoveBackward_member_overloads, 
									   Playground::MoveBackward, 0, 1);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TurnLeft_member_overloads, 
									   Playground::TurnLeft, 0, 1);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TurnRight_member_overloads, 
									   Playground::TurnRight, 0, 1);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(QueryObjectNearObject_member_overloads, 
									   Playground::QueryObjectNearObject, 1, 3);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateArena_member_overloads, 
									   Playground::CreateArena, 0, 2);

BOOST_PYTHON_MODULE(libxrobot)
{
	using namespace boost::python;

	class_<Range>("Range", init<boost::python::list, boost::python::list>())
	.add_property("GetMin", &Range::GetMin)
	.add_property("GetMax", &Range::GetMax)
	.def("__eq__", &Range::__eq__)
	.def("__ne__", &Range::__ne__)
	.def("__lt__", &Range::__lt__)
	.def("__gt__", &Range::__gt__)
	.def("__str__", &Range::__str__)
	;

	class_<Thing>("Thing", no_init)
	.def("GetPosition", &Thing::GetPosition)
	.def("GetOrientation", &Thing::GetOrientation)
	.def("GetLabel", &Thing::GetLabel)
	.def("__hash__", &Thing::__hash__)
	.def("__eq__", &Thing::__eq__)
	.def("__str__", &Thing::__str__)
	;

	class_<NavAgent>("NavAgent", no_init)
	.def("GetLabel", &NavAgent::GetLabel)
	.def("__eq__", &NavAgent::__eq__)
	.def("__str__", &NavAgent::__str__)
	;

	class_<Playground>("Playground", init<int,int,optional<int,int,int>>())

	.def("CreateArena", &Playground::CreateArena,
		CreateArena_member_overloads(
			args("width", "length"), "dimension"
		)
	)
	.def("EnableInventory", &Playground::EnableInventory,
		EnableInventory_member_overloads(
			args("max_capacity"), "capacity"
		)
	)
	.def("CreateEmptyScene", &Playground::CreateEmptyScene, 
		CreateEmptyScene_member_overloads(
			args("min_x", "max_x", "min_z", "max_z"), "range"
		)
	)
	.def("LoadSUNCG", &Playground::LoadSUNCG,
		LoadSUNCG_member_overloads(
			args("house", "metadata", "suncg_data_dir", "filter"), "suncg"
		)
	)
	.def("SpawnAnObject", &Playground::SpawnAnObject,
		SpawnAnObject_member_overloads(
			args("file", "position_py", "orentation_py", "scale", "label", "fixed", "occupy"),
			"spawn"
		)
	)
	.def("MoveForward", &Playground::MoveForward,
		MoveForward_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("MoveBackward", &Playground::MoveBackward,
		MoveBackward_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("TurnLeft", &Playground::TurnLeft,
		TurnLeft_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("TurnRight", &Playground::TurnRight,
		TurnRight_member_overloads(
			args("speed"), "speed"
		)
	)

	.def("QueryObjectNearObject", &Playground::QueryObjectNearObject,
		QueryObjectNearObject_member_overloads(
			args("near"), "dist"
		)
	)

	.def("CreateTerrain", &Playground::CreateTerrain,
		CreateTerrain_member_overloads(
			args("height", "terrain_size", "grid_size"), "terrain"
		)
	)

	.def("PaintPuddle", &Playground::PaintPuddle,
		PaintPuddle_member_overloads(
			args("layer_id"), "layer"
		)
	)

	.def("PaintCircle", &Playground::PaintCircle,
		PaintCircle_member_overloads(
			args("layer_id"), "layer"
		)
	)

	.def("PaintBox", &Playground::PaintBox,
		PaintBox_member_overloads(
			args("layer_id"), "layer"
		)
	)

	.def("PaintCurve", &Playground::PaintCurve,
		PaintPuddle_member_overloads(
			args("prec", "layer_id"), "layer"
		)
	)
	.def("LoadTerrain", &Playground::LoadTerrain)
	.def("ClearSpawnObjectsExceptAgent", &Playground::ClearSpawnObjectsExceptAgent)
	.def("HoldActions", &Playground::HoldActions)
	.def("UpdateAttachCamera", &Playground::UpdateAttachCamera)
	.def("ClearInventory", &Playground::ClearInventory)
	.def("RemoveAnObject", &Playground::RemoveAnObject)
	.def("GetAgent", &Playground::GetAgent)
	.def("GetXWorldScene", &Playground::GetXWorldScene)
	.def("DisplayInventory", &Playground::DisplayInventory)
	.def("LoadXWorldScene", &Playground::LoadXWorldScene)
	.def("HighlightCenter", &Playground::HighlightCenter)
	.def("QueryLastEvent", &Playground::QueryLastEvent)
	.def("GetCameraYaw", &Playground::GetCameraYaw)
	.def("GetSpaceNearPosition", &Playground::GetSpaceNearPosition)
	.def("ResolvePath", &Playground::ResolvePath)
	.def("GetGoals", &Playground::GetGoals)
	.def("LocatePositionInGrid", &Playground::LocatePositionInGrid)
	.def("LocateObjectInGrid", &Playground::LocateObjectInGrid)
	.def("GetRoomVisitSequence", &Playground::GetRoomVisitSequence)
	.def("QueryContact", &Playground::QueryContact)
	.def("GetRoomGroups", &Playground::GetRoomGroups)
	.def("LoadBasicObjects", &Playground::LoadBasicObjects)
	.def("OpenInventory", &Playground::OpenInventory)
	.def("CloseInventory", &Playground::CloseInventory)
	.def("Use", &Playground::Use)
	.def("AssignTag", &Playground::AssignTag)
	.def("LoadTag", &Playground::LoadTag)
	.def("MakeObjectPickable", &Playground::MakeObjectPickable)
	.def("GetWidth", &Playground::GetWidth)
	.def("GetHeight", &Playground::GetHeight)
	.def("SetLighting", &Playground::SetLighting)
	.def("EnableLidar", &Playground::EnableLidar)
	.def("UpdateLidar", &Playground::UpdateLidar)
	.def("EnableNavigation", &Playground::EnableNavigation)
	.def("AssignSurfaceLevel", &Playground::AssignSurfaceLevel)
	.def("AssignAgentRadius", &Playground::AssignAgentRadius)
	.def("BakeNavigationMesh", &Playground::BakeNavigationMesh)
	.def("AssignNavigationAgentTarget", &Playground::AssignNavigationAgentTarget)
	.def("SpawnNavigationAgent", &Playground::SpawnNavigationAgent)
	.def("Clear", &Playground::Clear)
	.def("CreateSceneFromSUNCG", &Playground::CreateSceneFromSUNCG)
	.def("CreateRandomGenerateScene", &Playground::CreateRandomGenerateScene)
	.def("LoadSceneConfigure", &Playground::LoadSceneConfigure)
	.def("SpawnModels", &Playground::SpawnModels)
	.def("SpawnModelsConf", &Playground::SpawnModelsConf)
	.def("LoadModels", &Playground::LoadModels)
	.def("AttachCameraTo", &Playground::AttachCameraTo)
	.def("FreeCamera", &Playground::FreeCamera)
	.def("UpdateFreeCamera", &Playground::UpdateFreeCamera)
	.def("Initialize", &Playground::Initialize)
	.def("UpdateSimulation", &Playground::UpdateSimulation)
	.def("UpdateRenderer", &Playground::UpdateRenderer)
	.def("Update", &Playground::Update)
	.def("LookUp", &Playground::LookUp)
	.def("LookDown", &Playground::LookDown)
	.def("Grasp", &Playground::Grasp)
	.def("PutDown", &Playground::PutDown)
	.def("Attach", &Playground::Attach)
	.def("Detach", &Playground::Detach)
	.def("Rotate", &Playground::Rotate)
	.def("TakeAction", &Playground::TakeAction)
	.def("ControlJointPositions", &Playground::ControlJointPositions)
	.def("ControlJointVelocities", &Playground::ControlJointVelocities)
	.def("Teleport", &Playground::Teleport)
	.def("GetCameraRGBDRaw", &Playground::GetCameraRGBDRaw)
	.def("QueryObjectAABBIntersect", &Playground::QueryObjectAABBIntersect)
	.def("QueryObjectWithLabelAtCameraCenter", &Playground::QueryObjectWithLabelAtCameraCenter)
	.def("QueryObjectWithLabelAtForward", &Playground::QueryObjectWithLabelAtForward)
	.def("QueryObjectWithLabelNearMe", &Playground::QueryObjectWithLabelNearMe)
	.def("QueryObjectAtCameraCenter", &Playground::QueryObjectAtCameraCenter)
	.def("QueryObjectByLabel", &Playground::QueryObjectByLabel)
	.def("UpdateSimulationWithAction", &Playground::UpdateSimulationWithAction)
	.def("GetObservationSpace", &Playground::GetObservationSpace)
	.def("GetActionSpace", &Playground::GetActionSpace)
	.def("GetCameraPosition", &Playground::GetCameraPosition)
	.def("GetCameraRight", &Playground::GetCameraRight)
	.def("GetCameraFront", &Playground::GetCameraFront)
	.def("GetCameraUp", &Playground::GetCameraUp)
	.def("GetStatus", &Playground::GetStatus)
	;

	scope().attr("ENABLE_INTERACTION")  = 11;
	scope().attr("DISABLE_INTERACTION") = 12;
	scope().attr("NO_ACTION")           = 14;
	scope().attr("HEADLESS")            = 1;
	scope().attr("VISUALIZATION")       = 0;
	scope().attr("HIDE")                = -1;
	scope().attr("GRID")                = 0;
	scope().attr("SUNCG")               = 1;
	scope().attr("REMOVE_NONE")         = -1;
	scope().attr("REMOVE_STAIR")        = 2;
	scope().attr("REMOVE_DOOR")         = 1;
	scope().attr("WORLD_UP")            = boost::python::make_tuple(0, 1, 0);
	scope().attr("METACLASS_WALL")      = std::string("Wall");
	scope().attr("METACLASS_FLOOR")     = std::string("Floor");
	scope().attr("METACLASS_CEILING")   = std::string("Ceiling");
	scope().attr("VERY_LOW")            = (int) render_engine::kVeryLow;
	scope().attr("LOW")                 = (int) render_engine::kLow;
	scope().attr("NORMAL")              = (int) render_engine::kNormal;
	scope().attr("MED")                 = (int) render_engine::kMed;
	scope().attr("HIGH")                = (int) render_engine::kHigh;
	scope().attr("EXTREME")             = (int) render_engine::kExtreme;
	scope().attr("SHADING")             = (int) render_engine::kShading;
	scope().attr("SHADOW")              = (int) render_engine::kShadow;
	scope().attr("AO")     	            = (int) render_engine::kAO;
	scope().attr("SSR")       			= (int) render_engine::kSR;
	scope().attr("VCT")                 = (int) render_engine::kVCT;
	scope().attr("AA")                  = (int) render_engine::kAA;
	scope().attr("GPU0")                = 0;
	scope().attr("GPU1")                = 1;
	scope().attr("GPU2")                = 2;
	scope().attr("GPU3")                = 3;
	scope().attr("GPU4")                = 4;
	scope().attr("GPU5")                = 5;
	scope().attr("GPU6")                = 6;
	scope().attr("GPU7")                = 7;
}
#endif // PLAYGROUND_PY_H_
