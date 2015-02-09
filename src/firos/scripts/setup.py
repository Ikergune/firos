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
import rospy
import traceback
import genmsg.msg_loader
from include.genpy import generator
from include.genpy import genpy_firos
from include import confManager
from include.logger import Log

def launchSetup(main=False):
    retcode = 0

    Log("INFO", "\nStarting Firos setup...")
    Log("INFO", "---------------------------------\n")
    Log("INFO", "\nGenerating Message Description Files\n")


    try:
        robots = confManager.getRobots(True, True)
        current_path = os.path.dirname(os.path.abspath(__file__))
        outdir = os.path.join(current_path, "include/ros/")
        retcode = genpy_firos.genmain(robots, generator.MsgGenerator(genmsg.msg_loader.load_msg_from_string), outdir)
        Log("INFO", "\nSuccesfully generated\n")
        if main:
            sys.exit(retcode or 0)
    except Exception as e:
        rospy.logerr("\nSomething wrong happened\n")
        traceback.print_exc()
        Log("ERROR", e)
        sys.exit(retcode or 0)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos_setup')
    launchSetup(True)

