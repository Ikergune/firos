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


def ros2Obj(msgInstance):
    ## \brief Map a Ros Object to dict
    # \param Ros Object instance
    obj = {}
    for key in msgInstance.__slots__:
        attr = getattr(msgInstance, key)
        if hasattr(attr, '__slots__'):
            obj[key] = ros2Obj(attr)
        else:
            obj[key] = attr
    return obj


def obj2Ros(obj, msgInstance):
    ## \brief Map a dict to Ros Object
    # \param dictionary
    # \param Ros Object instance
    if hasattr(msgInstance, '__slots__'):
        for key in msgInstance.__slots__:
            if key in obj:
                setattr(msgInstance, key, obj2Ros(obj[key], getattr(msgInstance, key)))
    else:
        if type(obj) is dict:
            raise Exception("Not a primitive")
        msgInstance = obj
    return msgInstance


def ros2Definition(msgInstance):
    ## \brief Generate Ros object definition
    # \param ROS Object instance
    obj = {}
    index = 0
    for key in msgInstance.__slots__:
        attr = getattr(msgInstance, key)
        if hasattr(attr, '__slots__'):
            obj[key] = ros2Definition(attr)
        else:
            obj[key] = msgInstance._slot_types[index]
        index += 1
    return obj
