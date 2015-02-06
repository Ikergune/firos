# LOGGING LEVELS
# 'CRITICAL', 'DEBUG', 'ERROR', 'INFO', 'WARNING'

import logging
import logging.handlers

from include.constants import LOGLEVEL

_logger = logging.getLogger('firos_logger')

if LOGLEVEL == 'CRITICAL':
    _logger.setLevel(logging.CRITICAL)
elif LOGLEVEL == 'ERROR':
    _logger.setLevel(logging.ERROR)
elif LOGLEVEL == 'WARNING':
    _logger.setLevel(logging.WARNING)
elif LOGLEVEL == 'DEBUG':
    _logger.setLevel(logging.DEBUG)
elif LOGLEVEL == 'INFO':
    _logger.setLevel(logging.INFO)

PRIORITIES = {
    'CRITICAL': 5,
    'ERROR': 3,
    'WARNING': 2,
    'DEBUG': 1,
    'INFO': 0
}
if LOGLEVEL == "NONE":
    _levelId = -1
else:
    _levelId = PRIORITIES[LOGLEVEL]

handler = logging.handlers.SysLogHandler(address = '/dev/log')

_logger.addHandler(handler)




def Log(level, *args):
    if PRIORITIES[level] <= _levelId:
        text = ""
        for arg in args:
            text = text + " " + str(arg)
        text = text[0:-1]
        if PRIORITIES[level] <= 2:
            print text
        else:
            if level == 'CRITICAL':
                _logger.critical(text)
            elif level == 'ERROR':
                print "error" + text
                _logger.error(text)
            elif level == 'WARNING':
                _logger.warning(text)
            elif level == 'DEBUG':
                print(text)
            elif level == 'INFO':
                print(text)
