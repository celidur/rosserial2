from .UInt32 import UInt32

class Time:
    def __init__(self):
        self.sec = UInt32()
        self.nanosec = UInt32()

    def serialize(self, message=None):
        if message is not None:
            self.sec.data = message.sec
            self.nanosec.data = message.nanosec
        return self.sec.serialize() + self.nanosec.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.sec.deserialize(data[offset:])
        offset += self.nanosec.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'sec': self.sec.data, 'nanosec': self.nanosec.data}

    def __set__(self, instance, value):
        self.nanosec.data = value.nanosec
        self.sec.data = value.sec

    def __hash__(self):
        return 0xcd7166c74c552c311fbcc2fe5a7bc289
