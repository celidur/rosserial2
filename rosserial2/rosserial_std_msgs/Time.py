from ..fuction import *


class Time:
    def __init__(self):
        self.sec: int = 0
        self.nanosec: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint32(self.sec) + serialization_uint32(self.nanosec)

    def deserialize(self, data, offset=0):
        offset, self.sec = deserialization_uint32(data, offset)
        offset, self.nanosec = deserialization_uint32(data, offset)
        return offset

    def __dict__(self):
        return {'sec': self.sec, 'nanosec': self.nanosec}

    def set(self, value):
        self.nanosec = value.nanosec
        self.sec = value.sec

    def __hash__(self):
        return 0xcd7166c74c552c311fbcc2fe5a7bc289
