#!/usr/bin/env python
# filepath: f:\study\软工\team05-project\Aft_g1\scripts\master_node.py

import rospy
import os
import re
import yaml
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from std_msgs.msg import String
from Aft_g1.msg import MappingCmd, NavControl
from Aft_g1.srv import MasterNode, MasterNodeResponse
import cv2
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
from pathlib import Path
from map_files import map_path, save_map

# 状态定义
STATE_IDLE = 0
STATE_NAVIGATION = 2
STATE_MAPPING = 3
STATE_ERROR = 4

# 指令类型定义
FORCE_STOP = 0
ROS_EXIT = 1

USER_CTRL_KEYBOARD = 12
USER_VOICE_CMD = 13

MAPPING_START = 20
MAPPING_END = 21
MAPPING_MOVE = 22
MAPPING_SAVE = 23

NAVIGATION_START = 30
NAVIGATION_END = 31
NAVIGATION_PATROL = 32
NAVIGATION_STOP = 34

PICK_TRIGGER = 40
PICK_ABORT = 41

state = STATE_IDLE
state_lock = threading.RLock()

# 全局变量定义
origin_x = 0.0
origin_y = 0.0
vel_pub = None
mapping_pub = None
navigation_pub = None
action_pub = None

# ========== 图片处理相关 ==========
latest_image = None
image_lock = threading.Lock()
bridge = CvBridge()

# 当前位置
current_pos = [0, 0, 0]
pos_lock = threading.Lock()

# 图片保存路径
IMG_SAVE_DIR = "/tmp/img"
MAX_SAVED_IMAGES = 3


# 图片回调，保存最新图片
def image_callback(msg):
    global latest_image
    with pos_lock:
        pos = current_pos.copy()
    with image_lock:
        latest_image = (msg, time.time_ns(), pos)


def save_latest_image(event=None):
    global latest_image
    with image_lock:
        sample = latest_image
        latest_image = None
    if sample is None:
        return
    msg, timestamp, pos = sample
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        directory = Path(IMG_SAVE_DIR)
        directory.mkdir(parents=True, exist_ok=True)
        filename = directory / f"{timestamp}_{pos[0]}_{pos[1]}_{pos[2]}.jpg"
        temporary = directory / f".{timestamp}.tmp.jpg"
        if not cv2.imwrite(str(temporary), cv_image):
            raise OSError("Could not encode camera image")
        os.replace(temporary, filename)
        for old in sorted(directory.glob("[0-9]*_*.jpg"), reverse=True)[MAX_SAVED_IMAGES:]:
            old.unlink(missing_ok=True)
    except Exception as e:
        rospy.logerr(f"[master_node] Error saving image: {e}")

# amcl_pose回调，更新当前位置
def amcl_pose_callback(msg):
    global current_pos
    with pos_lock:
        current_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]


def GetOrigin(file_name):
    """
    从指定yaml文件中读取地图原点（origin）
    使用PyYAML库解析yaml文件获取origin信息
    格式假设包含类似 "origin: [x, y, z]" 的行
    如果文件不存在或读取失败，仅警告并使用默认原点(0,0)
    """
    global origin_x, origin_y
    origin_x = 0.0
    origin_y = 0.0
    try:
        if not os.path.exists(file_name):
            rospy.logwarn(f"[debug] Missing map file: {file_name}, using default (0,0)")
            return
        with open(file_name, 'r') as f:
            map_data = yaml.safe_load(f)
            if 'origin' in map_data:
                origin_x = float(map_data['origin'][0])
                origin_y = float(map_data['origin'][1])
                rospy.loginfo("GetOrigin: x: %f, y: %f, z: %f", origin_x, origin_y, map_data['origin'][2])
            else:
                rospy.logwarn(f"[debug] No origin in {file_name}, using default (0,0)")
    except Exception as e:
        rospy.logwarn(f"[debug] Read origin error in {file_name}: {str(e)}, using default (0,0)")

def try_state_transition(valid_state, action_func, *args, **kwargs):
    """
    仅在当前状态允许该操作时执行，不修改被拒绝请求的状态。
    action_func: 目标操作函数
    """
    # return action_func(*args, **kwargs)
    global state
    if state != valid_state:
        rospy.logwarn(f"Rejected command in state {state}; expected {valid_state}")
        return False, None
    return action_func(*args, **kwargs)

def master_node_callback(req):
    with state_lock:
        return _master_node_callback(req)


def _master_node_callback(req):
    """
    回调函数，根据请求类型处理不同指令并通过各个话题发布消息
    """
    global state, origin_x, origin_y, mapping_pub, navigation_pub, action_pub
    res = MasterNodeResponse()
    rospy.loginfo(f"[debug][req_type] {req.type}")
    rospy.loginfo(f"[debug][cur_state] {state}")

    def mapping_start_action():
        global state
        nonlocal res
        old_state = state
        state = STATE_MAPPING
        rospy.loginfo(f"[debug] mapping_start_action, from {old_state} to {STATE_MAPPING}")
        res.code = 0
        msg = MappingCmd()
        msg.isCmd = True
        msg.cmdId = 1
        mapping_pub.publish(msg)
        rospy.loginfo("Starting mapping mode")
        return True, res

    def mapping_end_action():
        global state
        nonlocal res
        old_state = state
        state = STATE_IDLE
        rospy.loginfo(f"[debug] mapping_end_action, from {old_state} to {STATE_IDLE}")
        res.code = 0
        msg = MappingCmd()
        msg.isCmd = True
        msg.cmdId = 2
        mapping_pub.publish(msg)
        rospy.loginfo("Ending mapping mode")
        return True, res

    def mapping_move_action():
        nonlocal res
        rospy.loginfo(f"[debug] mapping_move_action, state={state}")
        msg = MappingCmd()
        msg.isCmd = False
        msg.direction = req.keyboard_ctrl_msg.direction
        msg.speed = req.keyboard_ctrl_msg.speed
        mapping_pub.publish(msg)
        rospy.loginfo("Mapping move: dir=%d, speed=%.2f", msg.direction, msg.speed)
        res.code = 0
        return True, res

    def mapping_save_action():
        nonlocal res
        rospy.loginfo(f"[debug] mapping_save_action, state={state}")
        if not req.navigation_ctrl_msg.name_list:
            raise ValueError("Map name is required")
        name = req.navigation_ctrl_msg.name_list[0]
        save_map(name, rospy.get_param("~maps_dir", "/home/robot/maps"))
        rospy.loginfo("Saved map: %s", name)
        res.code = 0
        return True, res

    def navigation_start_action():
        global state
        nonlocal res
        old_state = state
        msg = NavControl()
        msg.op = 0
        if not req.navigation_ctrl_msg.name_list:
            raise ValueError("Map name is required")
        msg.map_name = req.navigation_ctrl_msg.name_list[0]
        yaml_path = map_path(msg.map_name, rospy.get_param("~maps_dir", "/home/robot/maps"), ".yaml")
        if not yaml_path.is_file():
            raise ValueError("Map file does not exist")
        state = STATE_NAVIGATION
        res.code = 0
        rospy.loginfo(f"[debug] navigation_start_action, from {old_state} to {STATE_NAVIGATION} {msg.map_name}")
        yaml_name = str(yaml_path)
        GetOrigin(yaml_name)
        res.pose.position.x = origin_x
        res.pose.position.y = origin_y
        rospy.set_param('/map_origin', [origin_x, origin_y, 0.0])
        navigation_pub.publish(msg)
        rospy.loginfo("Starting navigation with map: %s", msg.map_name)
        return True, res

    def navigation_end_action():
        global state
        nonlocal res
        old_state = state
        state = STATE_IDLE
        rospy.loginfo(f"[debug] navigation_end_action, from {old_state} to {STATE_IDLE}")
        res.code = 0
        msg = NavControl()
        msg.op = 1
        navigation_pub.publish(msg)
        try:
            # os.system("rosnode kill sound_play 2>/dev/null")
            pass
        except:
            pass
        rospy.loginfo("Ending navigation mode")
        return True, res

    def navigation_patrol_action():
        nonlocal res
        rospy.loginfo(f"[debug] navigation_patrol_action, state={state}, waypoints={len(req.navigation_ctrl_msg.name_list)}")
        if not req.navigation_ctrl_msg.pose_list or len(req.navigation_ctrl_msg.pose_list) != len(req.navigation_ctrl_msg.name_list):
            raise ValueError("A nonempty path with matching names and poses is required")
        msg = NavControl()
        msg.op = 2
        msg.poses = req.navigation_ctrl_msg.pose_list
        msg.names = req.navigation_ctrl_msg.name_list
        msg.loop = req.navigation_ctrl_msg.loop
        navigation_pub.publish(msg)
        rospy.loginfo(f"[debug][Starting patrol with {len(msg.names)} waypoints]")
        res.code = 0
        return True, res

    def navigation_stop_action():
        nonlocal res
        rospy.loginfo(f"[debug] navigation_stop_action, state={state}")
        msg = NavControl()
        msg.op = 3
        navigation_pub.publish(msg)
        rospy.loginfo("Stopping navigation")
        res.code = 0
        return True, res

    def user_ctrl_keyboard_action():
        nonlocal res
        rospy.loginfo(f"[debug] user_ctrl_keyboard_action, state={state}")
        msg = NavControl()
        msg.direction = req.keyboard_ctrl_msg.direction
        if msg.direction >= 7:
            msg.op = 2
            rospy.logwarn(f"[debug] begin grab, direction={msg.direction}")
        else:
            msg.op = 1
        msg.speed = req.keyboard_ctrl_msg.speed
        action_pub.publish(msg)
        rospy.loginfo("Keyboard control: dir=%d, speed=%.2f", msg.direction, msg.speed)
        res.code = 0
        return True, res

    def user_voice_cmd_action():
        nonlocal res
        rospy.loginfo(f"[debug] user_voice_cmd_action, state={state}")
        msg = NavControl()
        msg.op = 5
        msg.command = req.command
        navigation_pub.publish(msg)
        rospy.loginfo("Voice command: %s", msg.command)
        res.code = 0
        return True, res

    def pick_trigger_action():
        nonlocal res
        rospy.loginfo(f"[debug] pick_trigger_action, state={state}")
        msg = NavControl()
        msg.op = 4
        msg.arm_op = 1
        navigation_pub.publish(msg)
        rospy.loginfo("Triggering pick operation")
        res.code = 0
        return True, res

    def pick_abort_action():
        nonlocal res
        rospy.loginfo(f"[debug] pick_abort_action, state={state}")
        msg = NavControl()
        msg.op = 4
        msg.arm_op = 0
        navigation_pub.publish(msg)
        rospy.loginfo("Aborting pick operation")
        res.code = 0
        return True, res

    try:
        if req.type == FORCE_STOP:
            state = STATE_IDLE
            navigation_pub.publish(NavControl(op=3))
            mapping_pub.publish(MappingCmd(isCmd=True, cmdId=2))
            action_pub.publish(NavControl(op=1, direction=0, speed=0))
            try:
                stop_msg = Twist()
                vel_pub.publish(stop_msg)
                res.code = 0
            except Exception as e:
                rospy.logerr("Error in FORCE_STOP: %s", str(e))
                res.code = 1
                res.msg = "处理FORCE_STOP错误: " + str(e)

        elif req.type == ROS_EXIT:
            state = STATE_IDLE
            navigation_pub.publish(NavControl(op=3))
            mapping_pub.publish(MappingCmd(isCmd=True, cmdId=2))
            action_pub.publish(NavControl(op=1, direction=0, speed=0))
            try:
                stop_msg = Twist()
                vel_pub.publish(stop_msg)
                res.code = 0
            except Exception as e:
                rospy.logerr("Error in ROS_EXIT: %s", str(e))
                res.code = 1
                res.msg = "处理ROS_EXIT错误: " + str(e)

        elif req.type == MAPPING_START:
            ok, r = try_state_transition(STATE_IDLE, mapping_start_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法进入建图模式"

        elif req.type == MAPPING_END:
            if state == STATE_IDLE:
                return res
            ok, r = try_state_transition(STATE_MAPPING, mapping_end_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法结束建图"

        elif req.type == MAPPING_MOVE:
            ok, r = try_state_transition(STATE_MAPPING, mapping_move_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法建图移动"

        elif req.type == MAPPING_SAVE:
            ok, r = try_state_transition(STATE_MAPPING, mapping_save_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法保存地图"

        elif req.type == NAVIGATION_START:
            ok, r = try_state_transition(STATE_IDLE, navigation_start_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法进入导航模式"

        elif req.type == NAVIGATION_END:
            if state == STATE_IDLE:
                return res
            ok, r = try_state_transition(STATE_NAVIGATION, navigation_end_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法结束导航"

        elif req.type == NAVIGATION_PATROL:
            ok, r = try_state_transition(STATE_NAVIGATION, navigation_patrol_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法巡逻"

        elif req.type == NAVIGATION_STOP:
            if state == STATE_IDLE:
                return res
            ok, r = try_state_transition(STATE_NAVIGATION, navigation_stop_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法停止导航"

        elif req.type == USER_CTRL_KEYBOARD:
            ok, r = try_state_transition(STATE_IDLE, user_ctrl_keyboard_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法键盘控制"

        elif req.type == USER_VOICE_CMD:
            # 语音指令支持IDLE和NAVIGATION
            if state not in [STATE_IDLE, STATE_NAVIGATION]:
                res.code = 1
                res.msg = "当前模式不接受语音导航"
                return res
            ok, r = user_voice_cmd_action()
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法语音控制"

        elif req.type == PICK_TRIGGER:
            ok, r = try_state_transition(STATE_NAVIGATION, pick_trigger_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法触发拾取"

        elif req.type == PICK_ABORT:
            ok, r = try_state_transition(STATE_NAVIGATION, pick_abort_action)
            if not ok:
                res.code = 1
                res.msg = "状态转移失败，无法中止拾取"

        else:
            res.code = 1
            res.msg = "未知指令"
            rospy.logwarn("Unknown command type: %d", req.type)

    except Exception as e:
        rospy.logerr("master_node_callback error: %s", str(e))
        res.code = 1
        res.msg = "主节点处理异常: " + str(e)

    return res

if __name__ == '__main__':
    try:
        rospy.init_node('master_node', anonymous=True)
        # 创建发布者
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        mapping_pub = rospy.Publisher("/map_ctrl", MappingCmd, queue_size=10)
        navigation_pub = rospy.Publisher("/nav", NavControl, queue_size=10)
        action_pub = rospy.Publisher("/act", NavControl, queue_size=10)

        IMG_SAVE_DIR = rospy.get_param("~image_dir", "/tmp/img")
        MAX_SAVED_IMAGES = max(1, int(rospy.get_param("~max_saved_images", 3)))
        rospy.Subscriber("/kinect2/hd/image_color_rect", Image, image_callback, queue_size=1, buff_size=16777216)
        rospy.Timer(rospy.Duration(max(0.1, float(rospy.get_param("~image_interval", 5)))), save_latest_image)
        # 订阅amcl_pose，获取机器人当前位置
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

        s = rospy.Service("/master_node", MasterNode, master_node_callback)
        rospy.loginfo("Master node started and ready to receive commands")
        rospy.spin()
    except Exception as e:
        rospy.logerr("Master node failed: %s", str(e))
