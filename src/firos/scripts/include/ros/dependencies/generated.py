unloaded = 0
libs=[]
try:
    import turtlebot_msgs.msg
except Exception:
    unloaded += 1
    libs.append('turtlebot_msgs')
try:
    import kobuki_msgs.msg
except Exception:
    unloaded += 1
    libs.append('kobuki_msgs')
try:
    import yocs_msgs.msg
except Exception:
    unloaded += 1
    libs.append('yocs_msgs')
try:
    import rocon_msgs.msg
except Exception:
    unloaded += 1
    libs.append('rocon_msgs')
try:
    import world_canvas_msgs.msg
except Exception:
    unloaded += 1
    libs.append('world_canvas_msgs')
try:
    import gazebo_msgs.msg
except Exception:
    unloaded += 1
    libs.append('gazebo_msgs')
try:
    import nav_msgs.msg
except Exception:
    unloaded += 1
    libs.append('nav_msgs')
try:
    import geometry_msgs.msg
except Exception:
    unloaded += 1
    libs.append('geometry_msgs')
try:
    import uuid_msgs.msg
except Exception:
    unloaded += 1
    libs.append('uuid_msgs')
try:
    import trajectory_msgs.msg
except Exception:
    unloaded += 1
    libs.append('trajectory_msgs')
try:
    import common_msgs.msg
except Exception:
    unloaded += 1
    libs.append('common_msgs')
try:
    import diagnostic_msgs.msg
except Exception:
    unloaded += 1
    libs.append('diagnostic_msgs')
try:
    import rosmsg.msg
except Exception:
    unloaded += 1
    libs.append('rosmsg')
try:
    import smach_msgs.msg
except Exception:
    unloaded += 1
    libs.append('smach_msgs')
try:
    import visualization_msgs.msg
except Exception:
    unloaded += 1
    libs.append('visualization_msgs')
try:
    import zeroconf_msgs.msg
except Exception:
    unloaded += 1
    libs.append('zeroconf_msgs')
try:
    import tf2_geometry_msgs.msg
except Exception:
    unloaded += 1
    libs.append('tf2_geometry_msgs')
try:
    import shape_msgs.msg
except Exception:
    unloaded += 1
    libs.append('shape_msgs')
try:
    import rocon_std_msgs.msg
except Exception:
    unloaded += 1
    libs.append('rocon_std_msgs')
try:
    import sensor_msgs.msg
except Exception:
    unloaded += 1
    libs.append('sensor_msgs')
try:
    import pcl_msgs.msg
except Exception:
    unloaded += 1
    libs.append('pcl_msgs')
try:
    import stdr_msgs.msg
except Exception:
    unloaded += 1
    libs.append('stdr_msgs')
try:
    import actionlib_msgs.msg
except Exception:
    unloaded += 1
    libs.append('actionlib_msgs')
try:
    import map_msgs.msg
except Exception:
    unloaded += 1
    libs.append('map_msgs')
try:
    import ar_track_alvar_msgs.msg
except Exception:
    unloaded += 1
    libs.append('ar_track_alvar_msgs')
try:
    import control_msgs.msg
except Exception:
    unloaded += 1
    libs.append('control_msgs')
try:
    import yocs_msgs.msg
except Exception:
    unloaded += 1
    libs.append('yocs_msgs')
try:
    import rocon_app_manager_msgs.msg
except Exception:
    unloaded += 1
    libs.append('rocon_app_manager_msgs')
try:
    import tf2_msgs.msg
except Exception:
    unloaded += 1
    libs.append('tf2_msgs')
try:
    import stereo_msgs.msg
except Exception:
    unloaded += 1
    libs.append('stereo_msgs')
try:
    import move_base_msgs.msg
except Exception:
    unloaded += 1
    libs.append('move_base_msgs')
try:
    import genmsg.msg
except Exception:
    unloaded += 1
    libs.append('genmsg')
try:
    import std_msgs.msg
except Exception:
    unloaded += 1
    libs.append('std_msgs')
try:
    import smart_battery_msgs.msg
except Exception:
    unloaded += 1
    libs.append('smart_battery_msgs')
try:
    import rqt_msg.msg
except Exception:
    unloaded += 1
    libs.append('rqt_msg')
try:
    import gateway_msgs.msg
except Exception:
    unloaded += 1
    libs.append('gateway_msgs')
try:
    import rocon_service_pair_msgs.msg
except Exception:
    unloaded += 1
    libs.append('rocon_service_pair_msgs')
try:
    import rosgraph_msgs.msg
except Exception:
    unloaded += 1
    libs.append('rosgraph_msgs')
print str(unloaded) + ' libraries not loaded: ' + str(libs)