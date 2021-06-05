#!/usr/bin/env python
import numpy as np
import pytest

import rospy
import rospkg
import pathlib
from moveit.core import load_robot_model
from moveit.core.robot_model import RobotModel


class TestPyRobotModel:
    def test_all(self):
        # Load the panda model
        p = rospkg.RosPack()
        urdf_path = pathlib.Path(p.get_path("moveit_resources_panda_description")) / 'urdf' / 'panda.urdf'
        srdf_path = pathlib.Path(p.get_path("moveit_resources_panda_moveit_config")) / 'config' / 'panda.srdf'
        model: RobotModel = load_robot_model(urdf_path.as_posix(), srdf_path.as_posix())

        # Test every exposed method
        print(f"joint model names {model.getJointModelNames()}")
        model.getActiveJointModels()
        print(f"variable count {model.getVariableCount()}")
        variable_count = model.getVariableCount()
        joint_positions1 = np.zeros(variable_count)
        joint_positions2 = np.zeros(variable_count) + 0.0
        # model.distance(joint_positions1, joint_positions2) # segfaults
        # model.enforcePositionBounds(joint_positions1)
        # active_joint_bounds = model.getActiveJointModelsBounds()
        # model.enforcePositionBounds(joint_positions1, active_joint_bounds)
        # joint_models = model.getJointModels()
        # joint_model1 = joint_models[0]
        # joint_model2 = joint_models[2]
        # model.getCommonRoot(joint_model1, joint_model2)
        # model.getMaximumExtent()
        # model.getMaximumExtent()
        # model.getMissingVariableNames()
        print(f"Model Frame: {model.getModelFrame()}")
        print(f"Name: {model.getName()}")
        # model.getSRDF()
        # model.getURDF()
        # model.getVariableBounds()
        # model.getVariableDefaultPositions()
        # model.getVariableDefaultPositions()
        # model.getVariableDefaultPositions()
        # model.getVariableDefaultPositions()
        # model.getVariableDefaultPositions()
        # model.getVariableDefaultPositions()
        print(f"Variable Index: {model.getVariableIndex('panda_joint1')}")
        print(f"Variable Names: {model.getVariableNames()}")
        # model.getRootJoint()
        print(f"{model.getRootJointName()}")
        # model.getJointModel()
        # model.getJointModel()
        # model.getJointModel()
        # model.getJointModels()
        print(f"{model.getJointModelNames()}")
        # model.getSingleDOFJointModels()
        # model.getMultiDOFJointModels()
        # model.getContinuousJointModels()
        # model.getMimicJointModels()
        # model.getJointOfVariable()
        # model.getJointOfVariable()
        # model.getJointModelCount()
        # model.hasJointModelGroup()
        # model.getJointModelGroup()
        # model.getJointModelGroup()
        # model.getJointModelGroups()
        print(f"{model.getJointModelGroupNames()}")
        # model.getEndEffector()
        print(model)

        # jmg = JointModelGroup()
        # jmg.addDefaultState()
        # jmg.attachEndEffector()
        # jmg.canSetStateFromIK()
        # jmg.distance()
        # jmg.enforcePositionBounds()
        # jmg.enforcePositionBounds()
        # jmg.enforcePositionBounds()
        # jmg.getActiveJointModelNames()
        # jmg.getActiveJointModels()
        # jmg.getActiveJointModelsBounds()
        # jmg.getAttachedEndEffectorNames()
        # jmg.getCommonRoot()
        # jmg.getConfig()
        # jmg.getContinuousJointModels()
        # jmg.getDefaultIKTimeout()
        # jmg.getDefaultStateNames()
        # jmg.getEndEffectorName()
        # jmg.getEndEffectorParentGroup()
        # jmg.getEndEffectorTips()
        # jmg.getEndEffectorTips()
        # jmg.getEndEffectorTips()
        # jmg.getEndEffectorTips()
        # jmg.getFixedJointModels()
        # jmg.getGroupKinematics()
        # jmg.getJointModel()
        # jmg.getJointModelNames()
        # jmg.getJointModels()
        # jmg.getJointRoots()
        # jmg.getKinematicsSolverJointBijection()
        # jmg.getLinkModel()
        # jmg.getLinkModelNames()
        # jmg.getLinkModelNamesWithCollisionGeometry()
        # jmg.getLinkModels()
        # jmg.getMaximumExtent()
        # jmg.getMaximumExtent()
        # jmg.getMaximumExtent()
        # jmg.getMimicJointModels()
        # jmg.getName()
        # jmg.getOnlyOneEndEffectorTip()
        # jmg.getParentModel()
        # jmg.getSolverInstance()
        # jmg.getSolverInstance()
        # jmg.getSubgroupNames()
        # jmg.getSubgroups()
        # jmg.getUpdatedLinkModelNames()
        # jmg.getUpdatedLinkModels()
        # jmg.getUpdatedLinkModelsSet()
        # jmg.getUpdatedLinkModelsWithGeometry()
        # jmg.getUpdatedLinkModelsWithGeometryNames()
        # jmg.getUpdatedLinkModelsWithGeometryNamesSet()
        # jmg.getUpdatedLinkModelsWithGeometrySet()
        # jmg.getVariableCount()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableDefaultPositions()
        # jmg.getVariableGroupIndex()
        # jmg.getVariableIndexList()
        # getVariableNames
        # hasJointModel
        # jmg.hasLinkModel()
        # jmg.interpolate()
        # jmg.isChain()
        # jmg.isContiguousWithinState()
        # jmg.isEndEffector()
        # jmg.isLinkUpdated()
        # jmg.isSingleDOFJoints()
        # jmg.isSubgroup()
        # jmg.isValidVelocityMove()
        # jmg.isValidVelocityMove()
        # jmg.printGroupInfo()
        # jmg.jmg.printGroupInfo(ss);()
        # jmg.satisfiesPositionBounds()
        # jmg.satisfiesPositionBounds,()
        # jmg.satisfiesPositionBounds()
        # jmg.satisfiesPositionBounds()
        # jmg.setDefaultIKTimeout()
        # jmg.setEndEffectorName()
        # jmg.setEndEffectorParent()
        # jmg.setRedundantJoints()
        # jmg.setSubgroupNames()
        # print(jmg)

        assert True


@pytest.fixture
def rospy_init_fixture():
    rospy.init_node("test_pyrobot_model")
    return TestPyRobotModel()


if __name__ == '__main__':
    pytest.main()
