#!/usr/bin/env python


# ROSPY LOGS
# logdebug
# logerr
# logfatal
# loginfo
# logout
# logwarn

# Import required Python code.
import os
import sys
import json
import rospy
import traceback
from include.genpy import generator
from include.genpy import genpy_main
from include import confManager

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos_setup')
    retcode = 0

    print "\nStarting Firos setup..."
    print "\nGenerating Messages\n"


    try:
        robots = confManager.getRobots()
        retcode = genpy_main.genmain(robots, generator.MsgGenerator())
        print "\nSuccesfully generated\n"
        sys.exit(retcode or 0)
    except Exception as e:
        rospy.logerr("\nSomething wrong happened\n")
        traceback.print_exc()
        print("ERROR: ",e)
        sys.exit(retcode or 0)

