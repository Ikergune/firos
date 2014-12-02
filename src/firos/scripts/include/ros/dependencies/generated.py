unloaded = 0
try:
    import turtlebot_msgs.msg
except Exception:
    unloaded += 1
try:
    import kobuki_msgs.msg
except Exception:
    unloaded += 1
try:
    import yocs_msgs.msg
except Exception:
    unloaded += 1
try:
    import rocon_msgs.msg
except Exception:
    unloaded += 1
try:
    import world_canvas_msgs.msg
except Exception:
    unloaded += 1
try:
    import gazebo_msgs.msg
except Exception:
    unloaded += 1
try:
    import nav_msgs.msg
except Exception:
    unloaded += 1
try:
    import geometry_msgs.msg
except Exception:
    unloaded += 1
try:
    import uuid_msgs.msg
except Exception:
    unloaded += 1
try:
    import trajectory_msgs.msg
except Exception:
    unloaded += 1
try:
    import common_msgs.msg
except Exception:
    unloaded += 1
try:
    import diagnostic_msgs.msg
except Exception:
    unloaded += 1
try:
    import rosmsg.msg
except Exception:
    unloaded += 1
try:
    import smach_msgs.msg
except Exception:
    unloaded += 1
try:
    import visualization_msgs.msg
except Exception:
    unloaded += 1
try:
    import zeroconf_msgs.msg
except Exception:
    unloaded += 1
try:
    import tf2_geometry_msgs.msg
except Exception:
    unloaded += 1
try:
    import shape_msgs.msg
except Exception:
    unloaded += 1
try:
    import rocon_std_msgs.msg
except Exception:
    unloaded += 1
try:
    import sensor_msgs.msg
except Exception:
    unloaded += 1
try:
    import pcl_msgs.msg
except Exception:
    unloaded += 1
try:
    import stdr_msgs.msg
except Exception:
    unloaded += 1
try:
    import actionlib_msgs.msg
except Exception:
    unloaded += 1
try:
    import map_msgs.msg
except Exception:
    unloaded += 1
try:
    import ar_track_alvar_msgs.msg
except Exception:
    unloaded += 1
try:
    import control_msgs.msg
except Exception:
    unloaded += 1
try:
    import yocs_msgs.msg
except Exception:
    unloaded += 1
try:
    import rocon_app_manager_msgs.msg
except Exception:
    unloaded += 1
try:
    import tf2_msgs.msg
except Exception:
    unloaded += 1
try:
    import stereo_msgs.msg
except Exception:
    unloaded += 1
try:
    import move_base_msgs.msg
except Exception:
    unloaded += 1
try:
    import genmsg.msg
except Exception:
    unloaded += 1
try:
    import std_msgs.msg
except Exception:
    unloaded += 1
try:
    import smart_battery_msgs.msg
except Exception:
    unloaded += 1
try:
    import rqt_msg.msg
except Exception:
    unloaded += 1
try:
    import gateway_msgs.msg
except Exception:
    unloaded += 1
try:
    import rocon_service_pair_msgs.msg
except Exception:
    unloaded += 1
try:
    import rosgraph_msgs.msg
except Exception:
    unloaded += 1
print unloaded