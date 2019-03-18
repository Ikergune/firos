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

regex = re.compile(u'^(.*)(\\b.msg\\b)(.*)$')

# TODO DL , loadFromFile and loadFromSystem is only used by Topic Handler
#           load3rdParty might be replaced by obj2ros?
class LibLoader:
    @staticmethod
    def loadFromFile(filepath):
        ## \brief Load file from path
        # \param File path
        mod_name, file_ext = os.path.splitext(os.path.split(filepath)[-1])

        if file_ext.lower() == '.py':
            py_mod = importlib.import_module(mod_name, package=filepath)

        elif file_ext.lower() == '.pyc':
            py_mod = importlib.import_module(mod_name, package=filepath)

        return py_mod

    @staticmethod
    def load3rdParty(filepath, className):
        ## \brief Load class from file
        # \param File path
        # \param Class name
        module = LibLoader.loadFromFile(filepath)
        return getattr(module, className)

    @staticmethod
    def loadFromSystem(lib):
        ## \brief Load library
        # \param library name
        module = None
        matches = re.search(regex, lib)
        if matches is not None:
            module_name = str(matches.group(1)) + str(matches.group(2))
            modules = str(matches.group(3)).split(".")
            modules = modules[1: len(modules)]
            module = importlib.import_module(module_name)

            
            for name in modules:
                module = getattr(module, name)
        return module
