#!/usr/bin/env python

from dji_sdk.srv import *
import rospy
import sys

srv_stereo_240p_name = '/dji_sdk/stereo_240p_subscription'
srv_stereo_depth_name = '/dji_sdk/stereo_depth_subscription'
srv_stereo_vga_name = '/dji_sdk/stereo_vga_subscription'
srv_main_and_fpv_name = '/dji_sdk/setup_camera_stream'

def node_log(string):
    rospy.loginfo("START_DJI_CAMERA_STREAM: " + string)

def node_error_log(string):
    rospy.logerr("START_DJI_CAMERA_STREAM: " + string)

def subscribe_to_240p(front_right, front_left, down_front, down_back, unsubscribe):
    try:
        subs_service = rospy.ServiceProxy(srv_stereo_240p_name, Stereo240pSubscription)
        node_log("Service call to 240p: front_right %d, front_left %d , down_front %d, down_back %d, unsubscribe %d" % 
            (front_right, front_left, down_front, down_back, unsubscribe))
        response = subs_service(front_right, front_left, down_front, down_back, unsubscribe)
    except rospy.ServiceException, e:
        node_error_log("Service call to 240p failed: %s"%e)
        return
    if response.result:
        node_log("Service call to 240p successful: %s" % response)
    else:
        node_error_log("Service call to 240p unsuccessful: %s" % response)

def subscribe_to_vga(subscribe, unsubscribe):
    try:
        node_log("Service call to vga: subscribe: %d, unsubscribe: %d" % (subscribe, unsubscribe))
        subs_service = rospy.ServiceProxy(srv_stereo_vga_name, StereoVGASubscription)
        response = subs_service(0, subscribe, unsubscribe) # always use 20Hz
    except rospy.ServiceException, e:
        node_error_log("Service call to vga failed: %s"%e)
        return

    if response.result:
        node_log("Service call to vga successful: %s" % response)
    else:
        node_error_log("Service call to vga unsuccessful: %s" % response)

# 0 for fpv, 1 for main camera
def subscribe_to_main_and_fpv(camera_type, start):
    try:
        node_log("Service call to main camera & fpv: camera_type: %d, start: %d" % (camera_type, start))
        subs_service = rospy.ServiceProxy(srv_main_and_fpv_name, SetupCameraStream)
        response = subs_service(camera_type, start)
    except rospy.ServiceException, e:
        node_error_log("Service call to main camera & fpv failed: %s"%e)
        return

    if response.result:
        node_log("Service call to main camera & fpv successful: %s" % response)
    else:
        node_error_log("Service call to main camera & fpv unsuccessful: %s" % response)

def main():

    use_240p = False
    if len(sys.argv) >= 2 and sys.argv[1] == "--down-stereo":
        node_log("Enable 240p topics, no front stereo vga(480p) subscription, instead front and down stereo are at 240p")
        use_240p = True


    rospy.init_node('start_dji_camera_stream');

    node_log("Waiting for service to start ...")
    rospy.wait_for_service('/dji_sdk/stereo_240p_subscription')
    rospy.wait_for_service('/dji_sdk/stereo_depth_subscription')
    rospy.wait_for_service('/dji_sdk/stereo_vga_subscription')
    rospy.wait_for_service('/dji_sdk/setup_camera_stream')
    node_log("All required service started\n")

    if use_240p:
        node_log("Enable subscription to 240p (all 4 cameras from 2 stereos) ...")
        subscribe_to_240p(1, 1, 1, 1, 0)
    else:
        node_log("Enable subscription to vga (2 camera from front stereo) ...")
        subscribe_to_vga(1, 0)

    node_log("Enable subscription to main gimbal camera ...")
    subscribe_to_main_and_fpv(0, 1)

    node_log("Enable subscription to front fpv camera ...")
    subscribe_to_main_and_fpv(1, 1)

    node_log("All done!")

if __name__ == "__main__":
    main()