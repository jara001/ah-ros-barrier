#!/usr/bin/env python3
# run.py
"""ROS1 Entrypoint for 'ah_ros_barrier'.
"""

import rospy

from ah_ros_bridge_example.module._run import RunNode


def main():
    """Starts a ROS node, registers the callbacks."""

    node = RunNode()

    # Function spin() simply keeps python from exiting until this node is stopped.
    rospy.spin()


if __name__ == '__main__':
    main()
