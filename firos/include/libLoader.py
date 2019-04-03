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

import os
import re
import imp
import importlib
import sys

# Add genpy to sys.path (This is not written as a module)
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/genpy/src/")
from genpy.generator import MsgGenerator
from include.logger import Log

regex = re.compile(u'^(.*)(\\b.msg\\b)(.*)$')


class LibLoader:
    ''' LibLoader is a class which tries to retrieve the python class of 
        specific ROS-Messages. First tries to retrieve it via the standard
        python import module. If this fails, it checks the folder FIORS/msgs for the 
        specific message. If the '.msg'-File is missing in the corresponding folder (or
        the class could not be generated), this LibLoader tries to load the message via
        roslib. If every method fails FIROS will shutdown, since FIROS need the messages
        due to rospy beforehand.
    '''

    # Our custom search path for genpy
    searchpath = dict()


    @staticmethod
    def _init_search_path(path):
        ''' Initializes the search path for genpy. 
            In this case we add all directory-names into the search path which
            are available in path.

            'namepace' still needs to be set to the actual package of the Message (TODO DL correct?)
        '''
        if len(LibLoader.searchpath) == 0:
            # Initialize seachpath
            subdirs = [x[0] for x in os.walk(path)] # get all directories inside path (including itself)
            subdirs = subdirs[1:] # Remove reference to itself
            for subdir in subdirs:
                subdir_name = subdir.split("/")[-1]
                LibLoader.searchpath[subdir_name] = [subdir]

        return LibLoader.searchpath


    @staticmethod
    def loadFromSystem(msgType, robotID, topic):
        ''' This actually tries all three methods mentioned above.

            Remark: If the regex does not find a match, we are also not able
                    to parse the Configuration-File ('robots.json') and exit.
        '''
        matches = re.search(regex, msgType) # get (PACKAGE).(msg).(MSGTYPE)

        if matches is not None:
            # We have a Match, we can start to get the Message now!
            module_name = str(matches.group(1)) + str(matches.group(2))[:-4] # PACKAGE + '.msg'
            modules = str(matches.group(3)).split(".") # MSGTYPE
            modules = modules[1: len(modules)]
            module_msg = modules[0]


            #####  1: Try to load it via Python-Import
            try:
                module = importlib.import_module(module_name + ".msg")
                clazz = getattr(module, module_msg)
                return clazz 
            except ImportError:
                Log("WARNING", "Message {} was not found. Trying to load the Message-File in FIROS/msgs".format(module_msg))



            ##### 2: Try to load the Message given the Message-files inside FIROS/msgs
            current_path = os.path.dirname(os.path.abspath(__file__))
            msgsFold = current_path + "/../../msgs/" # FIROS/msgs - Folder
            search_path = LibLoader._init_search_path(msgsFold)
            search_path["namespace"] = module_name
            try:    
                # Disable Output, since genpy prints exceptions 
                sys.stdout = open(os.devnull, "w")
                sys.stderr = open(os.devnull, "w")
                retcode = MsgGenerator().generate_messages(module_name, [msgsFold + module_name + "/" + module_msg + ".msg"], msgsFold + module_name, search_path)
            except Exception:
                # Case we got an Exception -> retcode = 1 -> Error
                retcode = 1
                pass
            finally:
                # Enable Output again
                sys.stdout = sys.__stdout__
                sys.stderr = sys.__stderr__

            if retcode == 1:
                Log("WARNING", "Could not load message {}/{}. Maybe it references other missing messages?".format(module_name, module_msg))
            elif retcode == 0:
                # TODO DL make Python3 compatible!
                Log("INFO", "Message {}/{}.msg succesfully loaded.".format(module_name, module_msg))
                module = imp.load_source(module_msg, msgsFold + module_name + "/_" + module_msg + ".py")
                clazz = getattr(module, module_msg)
                return clazz 



            ##### 3: Our last resort, the roslib.message which might know this message!
            try:
                import roslib.message
                import rostopic
                type_name = rostopic.get_topic_type('/{}/{}'.format(robotID, topic), blocking=False)[0]
                if type_name:
                    clazz = roslib.message.get_message_class(type_name)
                    return clazz
            except Exception:    
                pass


        ### Message could not be loaded or the regex does not match
        Log("ERROR", "Unable to load the message: {} on this system.".format(module_msg))
        exit()

