import os
from include.libLoader import LibLoader
THIRDPARTY_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "thirdparty")

# USE LibLoader.load3rdParty to assign a class to variable
# MyClass = LibLoader.load3rdParty("PATH/TO/myclass.py, 'MyClass')


# PLAY_MOTION_PATH = os.path.join(THIRDPARTY_BASE_PATH, 'play_motion_msgs/msg/')

# MotionInfo = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'MotionInfo.py'), 'MotionInfo')
# PlayMotionAction = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionAction.py'), 'PlayMotionAction')
# PlayMotionActionFeedback = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionActionFeedback.py'), 'PlayMotionActionFeedback')
# PlayMotionActionGoal = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionActionGoal.py'), 'PlayMotionActionGoal')
# PlayMotionActionResult = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionActionResult.py'), 'PlayMotionActionResult')
# PlayMotionFeedback = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionFeedback.py'), 'PlayMotionFeedback')
# PlayMotionGoal = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionGoal.py'), 'PlayMotionGoal')
# PlayMotionResult = LibLoader.load3rdParty(os.path.join(PLAY_MOTION_PATH, 'PlayMotionResult.py'), 'PlayMotionResult')
