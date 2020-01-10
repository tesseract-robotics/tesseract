from __future__ import absolute_import

def __preset_trajopt_log_level():
    import os
    if not 'TRAJOPT_LOG_THRESH' in os.environ:
        os.environ['TRAJOPT_LOG_THRESH'] = 'ERROR'
__preset_trajopt_log_level()

from .tesseract_python import *