import rospy
from Aft_g1.msg import NavControl
from Aft_g1.srv import MasterNode, MasterNodeRequest, MasterNodeResponse


def test_generated_topic_and_service_roundtrip(ros_master):
    publisher = rospy.Publisher("/test_nav_control", NavControl, queue_size=1, latch=True)
    service = rospy.Service("/test_master", MasterNode, lambda request: MasterNodeResponse(code=request.type, msg="roundtrip"))
    try:
        publisher.publish(NavControl(op=2, names=["first", "second"]))
        message = rospy.wait_for_message("/test_nav_control", NavControl, timeout=5)
        assert message.op == 2
        assert message.names == ["first", "second"]
        result = rospy.ServiceProxy("/test_master", MasterNode)(MasterNodeRequest(type=12))
        assert result.code == 12 and result.msg == "roundtrip"
    finally:
        publisher.unregister()
        service.shutdown()
