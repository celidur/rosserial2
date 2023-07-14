from ..fuction import *


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
        self.topic_id: int = 0
        self.topic_name: str = ""
        self.message_type: str = ""
        self.md5sum: str = ""
        self.buffer_size: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint16(self.topic_id) + \
            serialization_string(self.topic_name) + \
            serialization_string(self.message_type) + \
            serialization_string(self.md5sum) + \
            serialization_uint32(self.buffer_size)

    def deserialize(self, data, offset=0):
        offset, self.topic_id = deserialization_uint16(data, offset)
        offset, self.topic_name = deserialization_string(data, offset)
        offset, self.message_type = deserialization_string(data, offset)
        offset, self.md5sum = deserialization_string(data, offset)
        offset, self.buffer_size = deserialization_uint32(data, offset)
        return offset

    def __dict__(self):
        return {'topic_id': self.topic_id,
                'topic_name': self.topic_name,
                'message_type': self.message_type,
                'md5sum': self.md5sum,
                'buffer_size': self.buffer_size}

    def set(self, value):
        self.topic_id = value.topic_id
        self.topic_name = value.topic_name
        self.message_type = value.message_type
        self.md5sum = value.md5sum
        self.buffer_size = value.buffer_size

    def __hash__(self):
        return int("0x" + self.md5sum, 16)
