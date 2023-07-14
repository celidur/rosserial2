from .Time import Time
from ..fuction import *


class Header:
    def __init__(self):
        self.stamp: Time = Time()
        self.frame_id: str = ""

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint32(0) + self.stamp.serialize() + serialization_string(self.frame_id)

    def deserialize(self, data, offset=0):
        offset, _ = deserialization_uint32(data, offset)
        offset = self.stamp.deserialize(data, offset)
        offset, self.frame_id = deserialization_string(data, offset)
        return offset

    def __dict__(self):
        return {'stamp': self.stamp.__dict__(), 'frame_id': self.frame_id}

    def set(self, value):
        self.stamp.set(value.stamp)
        self.frame_id = value.frame_id

    def __hash__(self):
        return 0x2176decaecbce78abc3b96ef049fabed
