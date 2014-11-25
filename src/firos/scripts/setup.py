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

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos_setup')
    retcode = 0

    print "\nStarting Firos setup..."
    print "\nGenerating Messages\n"


    try:
        current_path = os.path.abspath(__file__)
        json_path = current_path.replace("scripts/setup.py", "config/robots.json")
        json_data = json.load(open(json_path))
        retcode = genpy_main.genmain(json_data, generator.MsgGenerator())
        print "\nSuccesfully generated\n"
        sys.exit(retcode or 0)
    except Exception as e:
        rospy.logerr("\nSomething wrong happened\n")
        traceback.print_exc()
        print("ERROR: ",e)
        sys.exit(retcode or 0)

