import rosserial2 as ros2
import rosserial2.std_msgs as std_msgs


class TopicInfo:
    ID_PUBLISHER = 0
    ID_SUBSCRIBER = 1
    ID_SERVICE_SERVER = 2
    ID_SERVICE_CLIENT = 4
    ID_PARAMETER_REQUEST = 6
    ID_LOG = 7
    ID_TIME = 10
    ID_TX_STOP = 11

    def __init__(self):
        self.topic_id = std_msgs.UInt16()
        self.topic_name = std_msgs.String()
        self.message_type = std_msgs.String()
        self.md5sum = std_msgs.String()
        self.buffer_size = std_msgs.UInt32()

    def serialize(self):
        ros2._logger.warning('it is not implemented')

    def deserialize(self, data):
        offset = 0
        offset += self.topic_id.deserialize(data[offset:])
        offset += self.topic_name.deserialize(data[offset:])
        offset += self.message_type.deserialize(data[offset:])
        offset += self.md5sum.deserialize(data[offset:])
        offset += self.buffer_size.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'topic_id': self.topic_id.data,
                'topic_name': self.topic_name.data,
                'message_type': self.message_type.data,
                'md5sum': self.md5sum.data,
                'buffer_size': self.buffer_size.data}

    def __set__(self, instance, value):
        self.topic_id.data = value.topic_id.data
        self.topic_name.data = value.topic_name.data
        self.message_type.data = value.message_type.data
        self.md5sum.data = value.md5sum.data
        self.buffer_size.data = value.buffer_size.data

    def __hash__(self):
        return int("0x" + self.md5sum.data, 16)
