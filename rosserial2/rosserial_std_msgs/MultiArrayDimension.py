from .String import String
from .UInt32 import UInt32


class MultiArrayDimension:
    def __init__(self):
        self.label = String()
        self.size = UInt32()
        self.stride = UInt32()

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return self.label.serialize() + self.size.serialize() + self.stride.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.label.deserialize(data[offset:])
        offset += self.size.deserialize(data[offset:])
        offset += self.stride.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {"label": self.label.data, "size": self.size.data, "stride": self.stride.data}

    def set(self, value):
        self.label.data = value.label
        self.size.data = value.size
        self.stride.data = value.stride

    def __hash__(self):
        return 0x4cd0c83a8683deae40ecdac60e53bfa8
