import struct


class UInt8:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('B', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('B', data[0:1])[0]
        return 1

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x7c8164229e7d2c17eb95e9231617fdee
