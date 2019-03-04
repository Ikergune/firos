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
import logging
import logging.handlers

from include.constants import Constants as C

_logger = logging.getLogger('firos_logger')

SYSLOG_ADDRESS = '/dev/log'

PRIORITIES = {
    'CRITICAL': 5,
    'ERROR': 3,
    'WARNING': 2,
    'DEBUG': 1,
    'INFO': 0
}

_levelId = None
handler = None

def initLog():
    ''' Sets _levelID and handler
    '''
    global _levelId, handler
    if C.LOGLEVEL == 'CRITICAL':
        _logger.setLevel(logging.CRITICAL)
    elif C.LOGLEVEL == 'ERROR':
        _logger.setLevel(logging.ERROR)
    elif C.LOGLEVEL == 'WARNING':
        _logger.setLevel(logging.WARNING)
    elif C.LOGLEVEL == 'DEBUG':
        _logger.setLevel(logging.DEBUG)
    elif C.LOGLEVEL == 'INFO':
        _logger.setLevel(logging.INFO)

    if C.LOGLEVEL == "NONE":
        _levelId = -1
    else:
        _levelId = PRIORITIES[C.LOGLEVEL]

    if os.path.exists(SYSLOG_ADDRESS):
        handler = logging.handlers.SysLogHandler(address=SYSLOG_ADDRESS)
        _logger.addHandler(handler)
    else:
        handler = None

def Log(level, *args):
    ## \brief Logging function
    # \param Log Level (INFO, DEBUG, WARNING, ERROR, CRITICAL)
    # \param Logging data
    if PRIORITIES[level] >= _levelId:
        text = ""
        for arg in args:
            text = text + " " + str(arg)
        text = text[1:]
        if handler is not None:
            if level == 'CRITICAL':
                _logger.critical(text)
            elif level == 'ERROR':
                _logger.error(text)
            elif level == 'WARNING':
                _logger.warning(text)
        print(text)
