/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Peter Mitrano
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The name of Peter Mitrano may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter Mitrano */

#include <pybind11/pybind11.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace py = pybind11;
using namespace planning_scene_monitor;

void def_planning_scene_monitor_bindings(py::module& m)
{
  py::class_<PlanningSceneMonitor, PlanningSceneMonitorPtr> psm(m, "PlanningSceneMonitor");

  py::enum_<PlanningSceneMonitor::SceneUpdateType>(psm, "SceneUpdateType")
      .value("UPDATE_NONE", PlanningSceneMonitor::SceneUpdateType::UPDATE_NONE)
      .value("UPDATE_STATE", PlanningSceneMonitor::SceneUpdateType::UPDATE_STATE)
      .value("UPDATE_TRANSFORMS", PlanningSceneMonitor::SceneUpdateType::UPDATE_TRANSFORMS)
      .value("UPDATE_GEOMETRY", PlanningSceneMonitor::SceneUpdateType::UPDATE_GEOMETRY)
      .value("UPDATE_SCENE", PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE)
      .export_values();

  psm.def(py::init([](const std::string& robot_description = "robot_description") {
       auto psm = std::make_shared<PlanningSceneMonitor>(robot_description);
       return psm;
     }))
      .def("addUpdateCallback",
           [&](PlanningSceneMonitor& self, py::function pyfunc) {
             auto py_update_fn = boost::function<void(PlanningSceneMonitor::SceneUpdateType)>(pyfunc);
             self.addUpdateCallback(py_update_fn);
           })
      .def("clearOctomap", &PlanningSceneMonitor::clearOctomap)
      .def("getDefaultAttachedObjectPadding", &PlanningSceneMonitor::getDefaultAttachedObjectPadding)
      .def("getDefaultObjectPadding", &PlanningSceneMonitor::getDefaultObjectPadding)
      .def("getDefaultRobotPadding", &PlanningSceneMonitor::getDefaultRobotPadding)
      .def("getDefaultRobotScale", &PlanningSceneMonitor::getDefaultRobotScale)
      .def("getLastUpdateTime", &PlanningSceneMonitor::getLastUpdateTime)
      .def("getMonitoredTopics", &PlanningSceneMonitor::getMonitoredTopics)
      .def("getName", &PlanningSceneMonitor::getName)
      .def("getPlanningSceneNonConst", py::overload_cast<>(&PlanningSceneMonitor::getPlanningScene))
      .def("getPlanningSceneConst", py::overload_cast<>(&PlanningSceneMonitor::getPlanningScene, py::const_))
      .def("getPlanningScenePublishingFrequency", &PlanningSceneMonitor::getPlanningScenePublishingFrequency)
      .def("getRobotDescription", &PlanningSceneMonitor::getRobotDescription)
      .def("getRobotModel", &PlanningSceneMonitor::getRobotModel)
      .def("getRobotModelLoader", &PlanningSceneMonitor::getRobotModelLoader)
      .def("getStateMonitor", &PlanningSceneMonitor::getStateMonitor)
      .def("getStateMonitorNonConst", &PlanningSceneMonitor::getStateMonitorNonConst)
      .def("getStateUpdateFrequency", &PlanningSceneMonitor::getStateUpdateFrequency)
      .def("lockSceneRead", &PlanningSceneMonitor::lockSceneRead)
      .def("lockSceneWrite", &PlanningSceneMonitor::lockSceneWrite)
      .def("monitorDiffs", &PlanningSceneMonitor::monitorDiffs)
      .def("newPlanningSceneMessage", &PlanningSceneMonitor::newPlanningSceneMessage)
      .def("providePlanningSceneService", &PlanningSceneMonitor::providePlanningSceneService)
      .def("publishDebugInformation", &PlanningSceneMonitor::publishDebugInformation)
      .def("requestPlanningSceneState", &PlanningSceneMonitor::requestPlanningSceneState)
      .def("setPlanningScenePublishingFrequency", &PlanningSceneMonitor::setPlanningScenePublishingFrequency)
      .def("setStateUpdateFrequency", &PlanningSceneMonitor::setStateUpdateFrequency)
      .def("startPublishingPlanningScene", &PlanningSceneMonitor::startPublishingPlanningScene)
      .def("startSceneMonitor", &PlanningSceneMonitor::startSceneMonitor)
      .def("startStateMonitor", &PlanningSceneMonitor::startStateMonitor)
      .def("startWorldGeometryMonitor", &PlanningSceneMonitor::startWorldGeometryMonitor)
      .def("stopPublishingPlanningScene", &PlanningSceneMonitor::stopPublishingPlanningScene)
      .def("stopSceneMonitor", &PlanningSceneMonitor::stopSceneMonitor)
      .def("stopStateMonitor", &PlanningSceneMonitor::stopStateMonitor)
      .def("stopWorldGeometryMonitor", &PlanningSceneMonitor::stopWorldGeometryMonitor)
      .def("triggerSceneUpdateEvent", &PlanningSceneMonitor::triggerSceneUpdateEvent)
      .def("unlockSceneRead", &PlanningSceneMonitor::unlockSceneRead)
      .def("unlockSceneWrite", &PlanningSceneMonitor::unlockSceneWrite)
      .def("updateFrameTransforms", &PlanningSceneMonitor::updateFrameTransforms)
      .def("updateSceneWithCurrentState", &PlanningSceneMonitor::updateSceneWithCurrentState)
      .def("updatesScene",
           py::overload_cast<const planning_scene::PlanningScenePtr&>(&PlanningSceneMonitor::updatesScene, py::const_))
      .def("waitForCurrentRobotState", &PlanningSceneMonitor::waitForCurrentRobotState)
      //
      ;
}
