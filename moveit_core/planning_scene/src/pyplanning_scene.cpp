#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/planning_scene/planning_scene.h>

namespace py = pybind11;
using namespace planning_scene;

auto CHECK_SELF_COLLISION1 =
    py::overload_cast<const collision_detection::CollisionRequest&,  //
                      collision_detection::CollisionResult&>(&PlanningScene::checkSelfCollision, py::const_);
auto CHECK_SELF_COLLISION2 = py::overload_cast<const collision_detection::CollisionRequest&,       //
                                               collision_detection::CollisionResult&,              //
                                               const robot_state::RobotState&,                     //
                                               const collision_detection::AllowedCollisionMatrix&  //
                                               >(&PlanningScene::checkSelfCollision, py::const_);

auto CHECK_COLLISION1 =
    py::overload_cast<const collision_detection::CollisionRequest&,  //
                      collision_detection::CollisionResult&>(&PlanningScene::checkCollision, py::const_);
auto CHECK_COLLISION2 = py::overload_cast<const collision_detection::CollisionRequest&,       //
                                          collision_detection::CollisionResult&,              //
                                          const robot_state::RobotState&,                     //
                                          const collision_detection::AllowedCollisionMatrix&  //
                                          >(&PlanningScene::checkCollision, py::const_);
auto IS_STATE_CONSTRAINED1 = py::overload_cast<const moveit_msgs::RobotState&,                        //
                                               const kinematic_constraints::KinematicConstraintSet&,  //
                                               bool>(&PlanningScene::isStateConstrained, py::const_);
auto IS_STATE_CONSTRAINED2 = py::overload_cast<const moveit::core::RobotState&,                       //
                                               const kinematic_constraints::KinematicConstraintSet&,  //
                                               bool>(&PlanningScene::isStateConstrained, py::const_);
auto IS_STATE_CONSTRAINED3 = py::overload_cast<const moveit_msgs::RobotState&,   //
                                               const moveit_msgs::Constraints&,  //
                                               bool>(&PlanningScene::isStateConstrained, py::const_);
auto IS_STATE_CONSTRAINED4 = py::overload_cast<const moveit::core::RobotState&,  //
                                               const moveit_msgs::Constraints&,  //
                                               bool>(&PlanningScene::isStateConstrained, py::const_);

auto GET_TRANSFORMS1 = py::overload_cast<>(&PlanningScene::getTransforms, py::const_);

auto IS_STATE_VALID1 = py::overload_cast<const moveit_msgs::RobotState&,  //
                                         const std::string&,              //
                                         bool>(&PlanningScene::isStateValid, py::const_);
auto IS_STATE_VALID2 = py::overload_cast<const moveit::core::RobotState&,  //
                                         const std::string&,               //
                                         bool>(&PlanningScene::isStateValid, py::const_);
auto IS_STATE_VALID3 = py::overload_cast<const moveit_msgs::RobotState&,   //
                                         const moveit_msgs::Constraints&,  //
                                         const std::string&,               //
                                         bool>(&PlanningScene::isStateValid, py::const_);
auto IS_STATE_VALID4 = py::overload_cast<const moveit::core::RobotState&,  //
                                         const moveit_msgs::Constraints&,  //
                                         const std::string&,               //
                                         bool>(&PlanningScene::isStateValid, py::const_);
auto IS_STATE_VALID5 = py::overload_cast<const moveit::core::RobotState&,                       //
                                         const kinematic_constraints::KinematicConstraintSet&,  //
                                         const std::string&,                                    //
                                         bool>(&PlanningScene::isStateValid, py::const_);

auto SET_CURRENT_STATE1 = py::overload_cast<const moveit_msgs::RobotState&>(&PlanningScene::setCurrentState);
auto SET_CURRENT_STATE2 = py::overload_cast<const robot_state::RobotState&>(&PlanningScene::setCurrentState);

void def_planning_scene_bindings(py::module& m)
{
  m.doc() = "The planning scene represents the state of the world and the robot, "
            "and can be used for collision checking";
  py::class_<PlanningScene, PlanningScenePtr>(m, "PlanningScene")
      .def(py::init<const moveit::core::RobotModelConstPtr&, const collision_detection::WorldPtr&>(),
           py::arg("robot_model"), py::arg("world") = collision_detection::WorldPtr(new collision_detection::World()))
      .def("checkSelfCollision", CHECK_SELF_COLLISION1)
      .def("checkSelfCollision", CHECK_SELF_COLLISION2)
      .def("checkCollision", CHECK_COLLISION1)
      .def("checkCollision", CHECK_COLLISION2)
      .def("getCurrentStateNonConst", &PlanningScene::getCurrentStateNonConst)
      .def("getCurrentState", &PlanningScene::getCurrentState)
      .def("getAllowedCollisionMatrix", &PlanningScene::getAllowedCollisionMatrix)
      .def("isStateConstrained", IS_STATE_CONSTRAINED1, py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained", IS_STATE_CONSTRAINED2, py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained", IS_STATE_CONSTRAINED3, py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained", IS_STATE_CONSTRAINED4, py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("getTransforms", GET_TRANSFORMS1, py::return_value_policy::reference)
      //      .def("setStateFeasibilityPredicate", &PlanningScene::setStateFeasibilityPredicate)
      .def("isStateValid", IS_STATE_VALID1, py::arg("state"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid", IS_STATE_VALID2, py::arg("state"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid", IS_STATE_VALID3, py::arg("state"), py::arg("constr"), py::arg("group"),
           py::arg("verbose") = false)
      .def("isStateValid", IS_STATE_VALID4, py::arg("state"), py::arg("constr"), py::arg("group"),
           py::arg("verbose") = false)
      .def("isStateValid", IS_STATE_VALID5, py::arg("state"), py::arg("constr"), py::arg("group"),
           py::arg("verbose") = false)
      .def("setCurrentState", SET_CURRENT_STATE1)
      .def("setCurrentState", SET_CURRENT_STATE2)
      //
      ;
}