import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src import common

def test_get_joint_names_returns_list():
    path = os.path.join(os.path.dirname(__file__), "..", "examples",
                         "synthetic_3dof_arm", "robot.urdf")
    names = common.get_joint_names(path)
    assert isinstance(names, list)
    assert len(names) == 3
