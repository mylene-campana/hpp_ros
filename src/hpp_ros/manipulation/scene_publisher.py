#!/usr/bin/env python

# Copyright (c) 2013 CNRS
# Author: Joseph Mirabel
#
# This file is part of hpp-ros.
# hpp-ros is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-ros is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-ros.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp_ros import ScenePublisher as BasicScenePublisher

class ScenePublisher (object):
    publishers = dict ()

    def __init__ (self, robot):
        self.deviceList = robot.client.manipulation.robot.getDeviceNames()
        self.ranks = dict ()
        total = 0
        for n in robot.jointNames:
            self.ranks [n] = total
            total += robot.getJointConfigSize (n)

        for d in self.deviceList:
            self.publishers [d] = BasicScenePublisher (
                robot,
                robot.client.manipulation.robot.getRootBody(robot.rootJointType[d],d),
                d,
                robot.client.manipulation.robot.getDeviceJointNames (d),
                self.ranks [d + "/" + robot.client.manipulation.robot.getDeviceJointNames (d)[0]]
                )

    def publish (self):
        for p in self.publishers.values():
            p (self.robotConfig)

    def publishRobots (self):
        self.publish ()

    def __call__ (self, args):
        self.robotConfig = args
        self.publish ()
