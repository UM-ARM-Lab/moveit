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
#include <pybind11/stl.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_model/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;
using namespace robot_model;

void def_robot_model_bindings(py::module& m)
{
  m.doc() = "Definition of a kinematic model. Not thread safe, however multiple instances can be created.";
  py::class_<RobotModel, RobotModelPtr>(m, "RobotModel")
      .def(py::init<const urdf::ModelInterfaceSharedPtr&, const srdf::ModelConstSharedPtr&>(), py::arg("urdf_model"),
           py::arg("srdf_model"))
      .def("getName", &RobotModel::getName)
      .def("getLinkModelNames", &RobotModel::getLinkModelNames)
      .def("getJointModelNames", &RobotModel::getJointModelNames)
      //
      ;

  py::class_<JointModelGroup, JointModelGroupPtr>(m, "JointModelGroup")
      .def(py::init<const std::string&, const srdf::Model::Group&, const std::vector<const JointModel*>&,
               const RobotModel*>(),
           py::arg("name"), py::arg("config"), py::arg("joint_vector"), py::arg("parent_model"))
      .def("addDefaultState", &JointModelGroup::addDefaultState)
      .def("attachEndEffector", &JointModelGroup::attachEndEffector)
      .def("canSetStateFromIK", &JointModelGroup::canSetStateFromIK)
      .def("distance", &JointModelGroup::distance)
      .def("enforcePositionBounds", py::overload_cast<double*>(&JointModelGroup::enforcePositionBounds, py::const_))
      .def("enforcePositionBounds",
           py::overload_cast<double*, const JointBoundsVector&>(&JointModelGroup::enforcePositionBounds, py::const_))
      .def("getActiveJointModelNames", &JointModelGroup::getActiveJointModelNames)
      .def("getActiveJointModels", &JointModelGroup::getActiveJointModels)
      .def("getActiveJointModelsBounds", &JointModelGroup::getActiveJointModelsBounds)
      .def("getAttachedEndEffectorNames", &JointModelGroup::getAttachedEndEffectorNames)
      .def("getCommonRoot", &JointModelGroup::getCommonRoot)
      .def("getConfig", &JointModelGroup::getConfig)
      .def("getContinuousJointModels", &JointModelGroup::getContinuousJointModels)
      .def("getDefaultIKTimeout", &JointModelGroup::getDefaultIKTimeout)
      .def("getDefaultStateNames", &JointModelGroup::getDefaultStateNames)
      .def("getEndEffectorName", &JointModelGroup::getEndEffectorName)
      .def("getEndEffectorParentGroup", &JointModelGroup::getEndEffectorParentGroup)
      .def("getEndEffectorTips",
           py::overload_cast<std::vector<const LinkModel*>&>(&JointModelGroup::getEndEffectorTips, py::const_))
      .def("getEndEffectorTips",
           py::overload_cast<std::vector<std::string>&>(&JointModelGroup::getEndEffectorTips, py::const_))
      .def("getFixedJointModels", &JointModelGroup::getFixedJointModels)
      .def("getGroupKinematics", &JointModelGroup::getGroupKinematics)
      .def("getJointModel", &JointModelGroup::getJointModel)
      .def("getJointModelNames", &JointModelGroup::getJointModelNames)
      .def("getJointModels", &JointModelGroup::getJointModels)
      .def("getJointRoots", &JointModelGroup::getJointRoots)
      .def("getKinematicsSolverJointBijection", &JointModelGroup::getKinematicsSolverJointBijection)
      .def("getLinkModel", &JointModelGroup::getLinkModel)
      .def("getLinkModelNames", &JointModelGroup::getLinkModelNames)
      .def("getLinkModelNamesWithCollisionGeometry", &JointModelGroup::getLinkModelNamesWithCollisionGeometry)
      .def("getLinkModels", &JointModelGroup::getLinkModels)
      .def("getMaximumExtent", py::overload_cast<>(&JointModelGroup::getMaximumExtent, py::const_))
      .def("getMaximumExtent",
           py::overload_cast<const JointBoundsVector&>(&JointModelGroup::getMaximumExtent, py::const_))
      .def("getMimicJointModels", &JointModelGroup::getMimicJointModels)
      .def("getName", &JointModelGroup::getName)
      .def("getOnlyOneEndEffectorTip", &JointModelGroup::getOnlyOneEndEffectorTip)
      .def("getParentModel", &JointModelGroup::getParentModel)
      .def("getSolverInstance", py::overload_cast<>(&JointModelGroup::getSolverInstance))
      .def("getSolverInstance", py::overload_cast<>(&JointModelGroup::getSolverInstance, py::const_))
      .def("getSubgroupNames", &JointModelGroup::getSubgroupNames)
      .def("getSubgroups", &JointModelGroup::getSubgroups)
      .def("getUpdatedLinkModelNames", &JointModelGroup::getUpdatedLinkModelNames)
      .def("getUpdatedLinkModels", &JointModelGroup::getUpdatedLinkModels)
      .def("getUpdatedLinkModelsSet", &JointModelGroup::getUpdatedLinkModelsSet)
      .def("getUpdatedLinkModelsWithGeometry", &JointModelGroup::getUpdatedLinkModelsWithGeometry)
      .def("getUpdatedLinkModelsWithGeometryNames", &JointModelGroup::getUpdatedLinkModelsWithGeometryNames)
      .def("getUpdatedLinkModelsWithGeometryNamesSet", &JointModelGroup::getUpdatedLinkModelsWithGeometryNamesSet)
      .def("getUpdatedLinkModelsWithGeometrySet", &JointModelGroup::getUpdatedLinkModelsWithGeometrySet)
      .def("getVariableCount", &JointModelGroup::getVariableCount)
      .def("getVariableDefaultPositions", py::overload_cast<const std::string&, std::map<std::string, double>&>(
          &JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<double*>(&JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<std::map<std::string, double>&>(&JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<std::vector<double>&>(&JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableGroupIndex", &JointModelGroup::getVariableGroupIndex)
      .def("getVariableIndexList", &JointModelGroup::getVariableIndexList)
      .def("getVariableNames", &JointModelGroup::getVariableNames)
      .def("getVariableRandomPositions", py::overload_cast<random_numbers::RandomNumberGenerator&, double*>(
          &JointModelGroup::getVariableRandomPositions, py::const_))
      .def("getVariableRandomPositions",
           py::overload_cast<random_numbers::RandomNumberGenerator&, double*, const JointBoundsVector&>(
               &JointModelGroup::getVariableRandomPositions, py::const_))
      .def("getVariableRandomPositions", py::overload_cast<random_numbers::RandomNumberGenerator&, std::vector<double>&>(
          &JointModelGroup::getVariableRandomPositions, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, double*, const double*, const double>(
               &JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, double*, const double*, const std::vector<double>&>(
               &JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, double*, const JointBoundsVector&, const double*,
               const double>(&JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, double*, const JointBoundsVector&, const double*,
               const std::map<JointModel::JointType, double>&>(
               &JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, double*, const JointBoundsVector&, const double*,
               const std::vector<double>&>(&JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, std::vector<double>&, const std::vector<double>&,
               const std::map<JointModel::JointType, double>&>(
               &JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, std::vector<double>&, const std::vector<double>&,
               const std::vector<double>&>(&JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("getVariableRandomPositionsNearBy",
           py::overload_cast<random_numbers::RandomNumberGenerator&, std::vector<double>&, const std::vector<double>&,
               double>(&JointModelGroup::getVariableRandomPositionsNearBy, py::const_))
      .def("hasJointModel", &JointModelGroup::hasJointModel)
      .def("hasLinkModel", &JointModelGroup::hasLinkModel)
      .def("interpolate", &JointModelGroup::interpolate)
      .def("isChain", &JointModelGroup::isChain)
      .def("isContiguousWithinState", &JointModelGroup::isContiguousWithinState)
      .def("isEndEffector", &JointModelGroup::isEndEffector)
      .def("isLinkUpdated", &JointModelGroup::isLinkUpdated)
      .def("isSingleDOFJoints", &JointModelGroup::isSingleDOFJoints)
      .def("isSubgroup", &JointModelGroup::isSubgroup)
      .def("isValidVelocityMove", py::overload_cast<const double*, const double*, std::size_t, double>(
          &JointModelGroup::isValidVelocityMove, py::const_))
      .def("isValidVelocityMove", py::overload_cast<const std::vector<double>&, const std::vector<double>&, double>(
          &JointModelGroup::isValidVelocityMove, py::const_))
      .def("printGroupInfo",
           [](const JointModelGroup& jmg) {
             std::stringstream ss;
             jmg.printGroupInfo(ss);
             return ss.str();
           })
      .def("satisfiesPositionBounds",
           py::overload_cast<const double*, const JointBoundsVector&, double>(&JointModelGroup::satisfiesPositionBounds,
                                                                              py::const_),
           py::arg("state"), py::arg("active_joint_bounds"), py::arg("margin") = 0.0)
      .def("satisfiesPositionBounds",
           py::overload_cast<const double*, double>(&JointModelGroup::satisfiesPositionBounds, py::const_),
           py::arg("state"), py::arg("margin") = 0.0)
      .def("setDefaultIKTimeout", &JointModelGroup::setDefaultIKTimeout)
      .def("setEndEffectorName", &JointModelGroup::setEndEffectorName)
      .def("setEndEffectorParent", &JointModelGroup::setEndEffectorParent)
      .def("setRedundantJoints", &JointModelGroup::setRedundantJoints)
      .def("setSubgroupNames", &JointModelGroup::setSubgroupNames)
      .def("__repr__",
           [](const JointModelGroup& jmg) {
             std::stringstream ss;
             jmg.printGroupInfo(ss);
             return ss.str();
           })
    //
      ;
}
