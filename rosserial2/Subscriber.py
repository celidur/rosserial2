from .RosType import RosType
import rclpy
import rosserial2 as ros2

class Subscriber:
    """
        Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = RosType(package, message)
        if hash(self.message) == hash(topic_info):
            self.subscriber = ros2._node.create_subscription(self.message.message_type, self.topic, self.callback, 10,
                                                             event_callbacks=rclpy.qos_event.SubscriptionEventCallbacks())
        else:
            ros2._logger.warning('This type is not implemented : ' + topic_info.message_type)

    def callback(self, msg):
        """ Forward message to serial device. """
        t = self.message.serialize(msg)
        if t is not None:
            self.parent.send(self.id, t)

    def unregister(self):
        ros2._logger.warning("Removing subscriber: " + self.topic)
        self.subscriber.unregister()