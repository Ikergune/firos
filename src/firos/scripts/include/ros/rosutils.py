def ros2Obj(msgInstance):
    obj = {}
    for key in msgInstance.__slots__:
        attr = getattr(msgInstance, key)
        if hasattr(attr, '__slots__'):
            obj[key] = ros2Obj(attr)
        else:
            obj[key] = attr
    return obj

def obj2Ros(obj, msgInstance):
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