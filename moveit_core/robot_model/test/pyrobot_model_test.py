#!/usr/bin/env python
import pathlib

import pytest
import rospkg
import rospy

from moveit.core import load_robot_model
from moveit.core.robot_model import RobotModel, RevoluteJointModel, FloatingJointModel, PlanarJointModel, \
    FixedJointModel, LinkModel


class TestPyRobotModel:
    def test_all(self):
        # Load the panda model
        p = rospkg.RosPack()
        urdf_path = pathlib.Path(p.get_path("moveit_resources_panda_description")) / 'urdf' / 'panda.urdf'
        srdf_path = pathlib.Path(p.get_path("moveit_resources_panda_moveit_config")) / 'config' / 'panda.srdf'
        model: RobotModel = load_robot_model(urdf_path.as_posix(), srdf_path.as_posix())

        # # Test every exposed method
        model.getActiveJointModelsBounds()
        model.getMaximumExtent()
        model.getMaximumExtent()
        model.getModelFrame()
        model.getName()
        model.getVariableBounds("panda_joint1")
        model.getVariableCount()
        model.getVariableDefaultPositions({"panda_joint1": 0.0})
        model.getVariableDefaultPositions([0.0])
        model.getVariableIndex("panda_joint1")
        model.getVariableNames()
        model.getRootJointName()
        link = model.getRootLink()

        joint = model.getRootJoint()
        # joint.addDescendantJointModel()
        # joint.addDescendantLinkModel()
        # joint.addMimicRequest()
        joint.getChildLinkModel()
        joint.getDescendantJointModels()
        joint.getDescendantLinkModels()
        joint.getDistanceFactor()
        joint.getMaximumExtent()
        joint.getMimic()
        joint.getMimicFactor()
        joint.getMimicOffset()
        joint.getMimicRequests()
        joint.getNonFixedDescendantJointModels()
        joint.getParentLinkModel()
        joint.getStateSpaceDimension()
        joint.getTypeName()
        joint.isPassive()
        # lifetime of these 'temporaries' is controlled in C++
        joint.setChildLinkModel(LinkModel("fake_child_link"))
        joint.setDistanceFactor(1)
        joint.setMimic(RevoluteJointModel("fake_mimic_joint"), 0, 0)
        joint.setParentLinkModel(LinkModel("fake_parent_link"))
        joint.setPassive(True)

        print(joint)

        model.getJointModelNames()
        model.getJointOfVariable("virtual_joint")
        model.getJointOfVariable(0)
        model.getJointModelCount()
        model.hasJointModelGroup("")
        print(model)

        link.getName()
        link.areCollisionOriginTransformsIdentity()
        link.getCenteredBoundingBoxOffset()
        link.getCollisionOriginTransforms()
        link.getFirstCollisionBodyTransformIndex()
        link.getJointOriginTransform()
        link.getLinkIndex()
        link.getShapeExtentsAtOrigin()
        link.getVisualMeshFilename()
        link.getVisualMeshOrigin()
        link.getVisualMeshScale()
        link.jointOriginTransformIsIdentity()
        link.parentJointIsFixed()

        assert True


@pytest.fixture
def rospy_init_fixture():
    rospy.init_node("test_pyrobot_model")
    return TestPyRobotModel()


if __name__ == '__main__':
    pytest.main()
