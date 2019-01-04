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

__author__ = "Dominik Lux"
__credits__ = ["Peter Detzner"]
__maintainer__ = "Dominik Lux"
__version__ = "0.0.1a"
__status__ = "Developement"

import importlib

def obj2RosViaStruct(obj, dataStruct):
    # TODO DL ERROR HANDLING! and refactor!
    if 'type' in dataStruct and 'value' in dataStruct:
        # Load Message-Class
        l = dataStruct['type'].split(".") # see Fiware-Object-Coverter, explicit Types of ROS-Messages are retreived from there 
        moduleLoader = importlib.import_module(l[0] + ".msg")
        msgClass = getattr(moduleLoader, l[1])
        instance = msgClass()
        for attr in msgClass.__slots__:
            if attr in obj and attr in dataStruct['value']:
                # Check if obj AND dataStruct contains attr
                if type(dataStruct['value'][attr]) is list:
                    l =[]
                    for it in range(len(dataStruct['value'][attr])):
                        l.append(obj2RosViaStruct(obj[attr][it], dataStruct['value'][attr][it]))
                    setattr(instance, attr, l)
                else:
                    setattr(instance, attr, obj2RosViaStruct(obj[attr], dataStruct['value'][attr]))
        return instance
    else:
        # Struct is {}:
        if type(obj) is dict:
            # if it is still a dict, convert into an Object with attributes
            t = Temp()
            for k in obj:
                setattr(t, k, obj[k])
            return t
        else:
            # something more simple (int, long, float), return it
            return obj

            
class Temp(object):
    pass

# TODO DL are Definitions still somewhere used?
def ros2Definition(msgInstance):
    ## \brief Generate Ros object definition
    # \param ROS Object instance
    obj = {}
    for key, t in zip(msgInstance.__slots__, msgInstance._slot_types):
        attr = getattr(msgInstance, key)
        if hasattr(attr, '__slots__'):
            obj[key] = ros2Definition(attr)
        else:
            obj[key] = t
    return obj
