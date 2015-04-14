#!/usr/bin/env python

# MIT License
#
# Copyright (c) <2015> <Ikergune, Etxetar>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


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
