#!/usr/bin/env python
PKG='test_arduino'
# import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest
import rospy

## A sample python unit test
class TestBareBones(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_two_notequals_one(self):
        self.assertNotEquals(2, 1, "2!=1")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_bare_bones', TestBareBones)