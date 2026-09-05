import threading
from unittest.mock import Mock

import pytest
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from Aft_g1.msg import NavControl
from Aft_g1.srv import MasterNodeRequest

import master_node
import map_files
from action_control import ActionControlNode
from nav_control_node import NavControlNode
from nav_node import NavController


@pytest.fixture
def master(monkeypatch):
    monkeypatch.setattr(master_node, "state", master_node.STATE_IDLE)
    for name in ("mapping_pub", "navigation_pub", "action_pub", "vel_pub"):
        monkeypatch.setattr(master_node, name, Mock())
    return master_node


def test_wrong_mode_does_not_reset_state_or_publish_manual_control(master):
    master.state = master.STATE_NAVIGATION
    request = MasterNodeRequest(type=master.USER_CTRL_KEYBOARD)
    response = master.master_node_callback(request)
    assert response.code != 0
    assert master.state == master.STATE_NAVIGATION
    master.action_pub.publish.assert_not_called()


def test_emergency_stop_cancels_navigation_and_manual_motion(master):
    master.state = master.STATE_NAVIGATION
    assert master.master_node_callback(MasterNodeRequest(type=master.FORCE_STOP)).code == 0
    assert master.state == master.STATE_IDLE
    assert master.navigation_pub.publish.call_args.args[0].op == 3
    assert master.action_pub.publish.call_args.args[0].direction == 0
    master.vel_pub.publish.assert_called_once()


def test_repeated_end_is_an_idempotent_noop(master):
    assert master.master_node_callback(MasterNodeRequest(type=master.NAVIGATION_END)).code == 0
    master.navigation_pub.publish.assert_not_called()


@pytest.mark.parametrize("name", ["../outside", "map;touch test", "a/b", "", "a\\b"])
def test_unsafe_map_name_never_reaches_a_process(name, tmp_path, monkeypatch):
    run = Mock()
    monkeypatch.setattr(map_files.subprocess, "run", run)
    with pytest.raises(ValueError):
        map_files.save_map(name, tmp_path)
    run.assert_not_called()


def test_map_saver_uses_arguments_and_a_timeout(tmp_path, monkeypatch):
    run = Mock()
    monkeypatch.setattr(map_files.subprocess, "run", run)
    map_files.save_map("map-1", tmp_path)
    assert run.call_args.args[0] == ["rosrun", "map_server", "map_saver", "-f", str(tmp_path / "map-1")]
    assert run.call_args.kwargs == {"check": True, "timeout": 4}


@pytest.fixture
def controller(monkeypatch):
    controller = NavController.__new__(NavController)
    controller.lock = threading.RLock()
    controller.wake = threading.Event()
    controller.generation = 0
    controller.status = 0
    controller.control = NavControl()
    controller.goal_timeout = 0.1
    controller.ac = Mock()
    controller.ac.wait_for_server.return_value = True
    controller.arrive_pub = Mock()
    controller.error_pub = Mock()
    monkeypatch.setattr(rospy, "is_shutdown", lambda: False)
    monkeypatch.setattr(rospy.Time, "now", lambda: rospy.Time(1))
    return controller


def path(loop=0):
    first, second = Pose(), Pose()
    first.position.x, second.position.x = 1.25, -2.5
    first.orientation.w = second.orientation.w = 1
    return NavControl(op=2, poses=[first, second], names=["first", "second"], loop=loop)


def test_single_pass_sends_each_world_coordinate_once(controller):
    controller.ac.get_state.return_value = GoalStatus.SUCCEEDED
    control = path()
    controller.nav_callback(control)
    controller._execute_path(controller.generation, control)
    goals = [call.args[0] for call in controller.ac.send_goal.call_args_list]
    assert [goal.target_pose.pose.position.x for goal in goals] == [1.25, -2.5]
    assert controller.arrive_pub.publish.call_count == 2
    assert controller.status == 0


@pytest.mark.parametrize("terminal", [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST])
def test_failed_or_cancelled_goal_never_advances_or_reports_arrival(controller, terminal):
    controller.ac.get_state.return_value = terminal
    control = path()
    controller.nav_callback(control)
    controller._execute_path(controller.generation, control)
    assert controller.ac.send_goal.call_count == 1
    controller.arrive_pub.publish.assert_not_called()
    assert controller.status == 0


def test_timeout_cancels_goal_without_waiting_forever(controller):
    controller.ac.get_state.return_value = GoalStatus.ACTIVE
    controller.goal_timeout = 0.01
    control = path()
    controller.nav_callback(control)
    controller._execute_path(controller.generation, control)
    controller.ac.cancel_goal.assert_called_once()
    assert controller.status == 0


def test_stop_during_goal_prevents_next_waypoint(controller):
    def stopped():
        controller.sub_callback(String())
        return GoalStatus.SUCCEEDED
    controller.ac.get_state.side_effect = stopped
    control = path()
    controller.nav_callback(control)
    controller._execute_path(controller.generation, control)
    assert controller.ac.send_goal.call_count == 1
    controller.arrive_pub.publish.assert_not_called()


def test_loop_repeats_until_stopped(controller):
    count = 0
    def state():
        nonlocal count
        count += 1
        if count == 3:
            controller.sub_callback(String())
        return GoalStatus.SUCCEEDED
    controller.ac.get_state.side_effect = state
    control = path(loop=1)
    controller.nav_callback(control)
    controller._execute_path(controller.generation, control)
    assert controller.ac.send_goal.call_count == 3
    assert controller.status == 0


def test_navigation_exit_only_stops_and_never_sends_a_return_goal():
    node = NavControlNode.__new__(NavControlNode)
    node.ctrl_pub_stop, node.ctrl_pub_vel, node.ctrl_pub_nav = Mock(), Mock(), Mock()
    node.status = 1
    node.handle_stop_navigation()
    assert node.status == 0
    node.ctrl_pub_stop.publish.assert_called_once()
    node.ctrl_pub_vel.publish.assert_called_once()
    node.ctrl_pub_nav.publish.assert_not_called()


def test_manual_stop_is_accepted_during_grabbing():
    node = ActionControlNode.__new__(ActionControlNode)
    node.is_grab = True
    node.grab_cancel = threading.Event()
    node.ctrl_pub_vel = Mock()
    node.act_callback(NavControl(op=1, direction=0, speed=0))
    assert node.grab_cancel.is_set()
    command = node.ctrl_pub_vel.publish.call_args.args[0]
    assert command.linear.x == 0 and command.angular.z == 0
