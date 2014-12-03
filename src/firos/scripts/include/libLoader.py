import os
import imp
import pkgutil

class LibLoader:
    @staticmethod
    def loadFromFile(filepath):
        mod_name,file_ext = os.path.splitext(os.path.split(filepath)[-1])

        if file_ext.lower() == '.py':
            py_mod = imp.load_source(mod_name, filepath)

        elif file_ext.lower() == '.pyc':
            py_mod = imp.load_compiled(mod_name, filepath)

        return py_mod
    @staticmethod
    def loadFromSystem(lib):
        modules = lib.split(".")
        module = __import__(modules.pop(0))
        for name in modules:
            module = getattr(module, name)
        return module

def generateRosDependencies():
    directories = os.environ["ROS_PACKAGE_PATH"].split(":")
    imports = """unloaded = 0\nlibs=[]\n"""
    for directory in directories:
        if os.path.isdir(directory):
            for folder in os.listdir(directory):
                if "msg" in folder:
                    imports += """try:\n    import {}.msg\nexcept Exception:\n    unloaded += 1\n    libs.append('{}')\n""".format(folder,folder)
    imports += "print str(unloaded) + ' libraries not loaded: ' + str(libs)"
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros", "dependencies", "generated.py")
    f = open(path,'w')
    f.write(imports)
    f.close()