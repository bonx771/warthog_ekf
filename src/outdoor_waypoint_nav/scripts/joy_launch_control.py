#!/usr/bin/env python3
import rospy
import roslaunch
import rospkg
import os
import signal
import subprocess
import sys
import threading
import textwrap
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Initialize variables

buttons_array = [0, 0, 0, 0, 0]
collect_btn_num = 0
collect_btn_sym = ""
send_btn_num = 0
send_btn_sym = ""
calibrate_btn_num = 0
calibrate_btn_sym = ""
abort_btn_num = 0
abort_btn_sym = ""
sim_enabled = False

location_collect = ""
location_send = ""
location_calibrate = ""
location_safety_node = ""

calibrate_complete = False
collect_complete = False
send_complete = False
velocity_paused = False
launch_process = None
launch_label = ""
launch_output_thread = None

def getParameter():
    global collect_btn_num
    global collect_btn_sym
    global send_btn_num
    global send_btn_sym
    global calibrate_btn_num
    global calibrate_btn_sym
    global abort_btn_num
    global abort_btn_sym
    global continue_btn_num
    global continue_btn_sym
    global sim_enabled

    collect_btn_num = rospy.get_param("/outdoor_waypoint_nav/collect_button_num")
    collect_btn_sym = rospy.get_param("/outdoor_waypoint_nav/collect_button_sym")
    send_btn_num = rospy.get_param("/outdoor_waypoint_nav/send_button_num")
    send_btn_sym = rospy.get_param("/outdoor_waypoint_nav/send_button_sym")
    calibrate_btn_num = rospy.get_param("/outdoor_waypoint_nav/calibrate_button_num")
    calibrate_btn_sym = rospy.get_param("/outdoor_waypoint_nav/calibrate_button_sym")
    abort_btn_num = rospy.get_param("/outdoor_waypoint_nav/abort_button_num")
    abort_btn_sym = rospy.get_param("/outdoor_waypoint_nav/abort_button_sym")
    continue_btn_num = rospy.get_param("/outdoor_waypoint_nav/continue_button_num")
    continue_btn_sym = rospy.get_param("/outdoor_waypoint_nav/continue_button_sym")
    
    sim_enabled = rospy.get_param("/outdoor_waypoint_nav/sim_enabled")

def getPaths():
    global location_collect
    global location_send
    global location_calibrate
    global location_safety_node
    rospack = rospkg.RosPack()
    
    # Define location of launch files
    if sim_enabled == True:
        location_collect = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/collect_goals_sim.launch"
        location_send = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/send_goals_sim.launch"
        location_calibrate = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/heading_calibration_sim.launch"
        location_safety_node = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/safety_node.launch"

    elif sim_enabled == False:
        location_collect = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/collect_goals.launch"
        location_send = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/send_goals.launch"
        location_calibrate = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/heading_calibration.launch"
        location_safety_node = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/safety_node.launch"

    else:
        print("ERROR: PLEASE SPECIFY SIM_ENABLED PARAMETER.")

def joy_CB(joy_msg):
    global start_collect_btn
    global buttons_array 
    buttons_array = [joy_msg.buttons[collect_btn_num],joy_msg.buttons[send_btn_num],joy_msg.buttons[calibrate_btn_num], joy_msg.buttons[abort_btn_num], joy_msg.buttons[continue_btn_num]]

def calibrate_status_CB(calibrate_status_msg):
    global calibrate_complete
    calibrate_complete = calibrate_status_msg.data

def collection_status_CB(collection_status_msg):
    global collect_complete
    collect_complete = collection_status_msg.data

def waypoint_following_status_CB(waypoint_following_status_msg):
    global send_complete
    send_complete = waypoint_following_status_msg.data

def launch_subscribers():
    rospy.init_node('joy_launch_control')
    rospy.Subscriber("/joy_teleop/joy",Joy, joy_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/calibrate_status",Bool, calibrate_status_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/collection_status",Bool, collection_status_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/waypoint_following_status",Bool, waypoint_following_status_CB )

def print_instructions():
    instructions = textwrap.dedent(
        """
        ---------------- Waypoint Control ----------------
        Press {collect} to start waypoint collection
        Press {send} to start waypoint following
        Press {calibrate} to perform heading calibration
        Press {abort} at any time to stop robot motion
        --------------------------------------------------
        """
    ).format(
        collect=collect_btn_sym,
        send=send_btn_sym,
        calibrate=calibrate_btn_sym,
        abort=abort_btn_sym,
    ).strip("\n")
    sys.stdout.write("\n" + instructions + "\n\n")
    sys.stdout.flush()

def relay_launch_output(process, label):
    for raw_line in iter(process.stdout.readline, ""):
        stripped = raw_line.strip()
        if not stripped:
            continue

        # Show only actionable node output and hide roslaunch boilerplate.
        if (
            "[INFO]" in stripped
            or "[WARN]" in stripped
            or "[ERROR]" in stripped
            or stripped.startswith("Press ")
        ):
            sys.stdout.write(stripped + "\n")
            sys.stdout.flush()

    process.stdout.close()

def shutdown_launch_process():
    global launch_process
    global launch_label

    if launch_process is None:
        return

    if launch_process.poll() is None:
        try:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
        except OSError:
            pass
    launch_process = None
    launch_label = ""

def start_launch_process(launch_file, label):
    global launch_process
    global launch_label
    global launch_output_thread

    shutdown_launch_process()
    rospy.loginfo("Starting %s...", label)
    launch_process = subprocess.Popen(
        ["roslaunch", launch_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )
    launch_label = label
    launch_output_thread = threading.Thread(
        target=relay_launch_output,
        args=(launch_process, label),
        daemon=True,
    )
    launch_output_thread.start()

def get_coordinates_file_path():
    path_local = rospy.get_param("/outdoor_waypoint_nav/coordinates_file", None)
    if path_local:
        return rospkg.RosPack().get_path("outdoor_waypoint_nav") + path_local

    filename = "/waypoint_files/points_sim.txt" if sim_enabled else "/waypoint_files/points_outdoor.txt"
    return rospkg.RosPack().get_path("outdoor_waypoint_nav") + filename

def count_waypoint_tokens(filepath):
    if not os.path.exists(filepath):
        return 0

    with open(filepath, "r", encoding="utf-8") as waypoint_file:
        return len(waypoint_file.read().split())

def has_move_base_server():
    published_topics = dict(rospy.get_published_topics())
    required_topics = [
        "/move_base/status",
        "/move_base/goal",
        "/move_base/result",
    ]
    return all(topic in published_topics for topic in required_topics)

def check_buttons():

    global buttons_array     
    global launch 
    global calibrate_complete
    global collect_complete
    global send_complete
    global velocity_paused
    
    # Check abort button
    if buttons_array[3] == 1:
        rospy.logerr("STOP BUTTON SELECTED, blocking velocity commands...")
        os.system("rosnode kill safety_node")
        rospy.sleep(1) # Sleep for 1 second to allow time for node to shutdown
        sys.stdout.write("\nPress %s to continue following waypoints\n\n" % continue_btn_sym)
        sys.stdout.flush()
        velocity_paused = True
    
    elif buttons_array[4] == 1 and velocity_paused == True:
        rospy.loginfo("continuing to follow wapoints...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[location_safety_node])
        launch.start()
        velocity_paused = False

    # Start collecting goals
    if buttons_array[0] == 1:
        while buttons_array[0] == 1:    # Wait for button to be released
            pass
        start_launch_process(location_collect, "collect_goals.launch")

    # Start sending goals
    elif buttons_array[1] == 1:
        while buttons_array[1] ==1:
            pass
        if not has_move_base_server():
            rospy.logerr("move_base is not running. Start outdoor_waypoint_nav_sim.launch before pressing %s.", send_btn_sym)
            return

        waypoint_path = get_coordinates_file_path()
        waypoint_token_count = count_waypoint_tokens(waypoint_path)
        if waypoint_token_count < 2:
            rospy.logerr("No waypoint available in %s. Collect waypoint(s) before pressing %s.", waypoint_path, send_btn_sym)
            return

        rospy.loginfo("Using waypoint file: %s", waypoint_path)
        rospy.loginfo("Waypoint count detected: %d", waypoint_token_count // 2)
        start_launch_process(location_send, "send_goals.launch")

    # Start Heading Calbration
    elif buttons_array[2] == 1:
        while buttons_array[2] ==1:
            pass
        if sim_enabled:
            rospy.logwarn("Heading calibration is disabled in simulation. Using navsat_params_sim.yaml defaults instead.")
            return
        start_launch_process(location_calibrate, "heading_calibration.launch")

    # Check if end notice has been published by other nodes
    if (calibrate_complete or collect_complete or send_complete):
        rospy.sleep(2) # Sleep for 2 seconds to allow time for other nodes to shutdown
        shutdown_launch_process()
        print_instructions()
        # Reset all parameters
        calibrate_complete = False
        collect_complete = False
        send_complete = False

def main():

    # start node to subscribe to joy messages node end messages 
    launch_subscribers()
    rospy.on_shutdown(shutdown_launch_process)

    # check buttons and launch the appropriate file
    while not rospy.is_shutdown():
        check_buttons()
    rospy.spin()

if __name__ == '__main__':

    getParameter()
    getPaths()

    print_instructions()

    if sim_enabled == False:
        sys.stdout.write(
            "NOTE: It is recommended to perform one or two heading calibrations\n"
            "      each time the robot is starting from a new heading.\n"
        )
        sys.stdout.flush()
    
    main()
    
