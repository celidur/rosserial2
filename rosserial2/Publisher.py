from .RosType import RosType
import rclpy
import rosserial2 as ros2

class Publisher:
    """
        Publisher forwards messages from the serial device to ROS.
    """

    def __init__(self, topic_info):
        """ Create a new publisher. """
        self.topic = topic_info.topic_name.data
        # find message type
        package, message = topic_info.message_type.data.split('/')
        # self.manage = getRosType(package, message, topic_info.message_type)
        self.message = RosType(package, message)

        if hash(self.message) == hash(topic_info):
            self.publisher = ros2._node.create_publisher(
                self.message.message_type,
                self.topic,
                rclpy.qos.QoSProfile(depth=10, history=rclpy.qos.HistoryPolicy.KEEP_LAST)
            )
        else:
            ros2._logger.warning('This type is not implemented : ' + topic_info.message_type.data)

    def handlePacket(self, data):
        """ Forward message to ROS network. """
        self.publisher.publish(self.message.deserialize(data))
