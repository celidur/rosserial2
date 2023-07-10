from .String import String
from .Time import Time
from .UInt32 import UInt32

class Header:
    def __init__(self):
        self.seq = UInt32()
        self.stamp = Time()
        self.frame_id = String()

    def serialize(self, message=None):
        if message is None:
            message = self
        self.seq = message.seq
        self.stamp = message.stamp
        self.frame_id = message.frame_id
        return self.seq

    def deserialize(self, data):
        offset = 0
        offset += self.seq.deserialize(data[offset:])
        offset += self.stamp.deserialize(data[offset:])
        offset += self.frame_id.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'stamp': self.stamp.__dict__(), 'frame_id': self.frame_id.data}

    def __set__(self, instance, value):
        self.stamp = value.stamp
        self.frame_id = value.frame_id

    def __hash__(self):
        return 0x2176decaecbce78abc3b96ef049fabed
