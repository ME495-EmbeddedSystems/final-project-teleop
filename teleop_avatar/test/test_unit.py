"""Test quaternion handling."""

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np


def test_quaternion_handling():
    tf = TransformStamped()

    x = tf.transform.rotation.x
    y = tf.transform.rotation.y
    z = tf.transform.rotation.z
    w = tf.transform.rotation.w

    assert quaternion_from_matrix(np.array(quaternion_matrix([x, y, z, w]))) == (
        x,
        y,
        z,
        w,
    )
