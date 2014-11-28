import os
import imp

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
