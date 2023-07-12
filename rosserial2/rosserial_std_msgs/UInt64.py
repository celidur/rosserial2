import struct


class UInt64:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('Q', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('Q', data[0:8])[0]
        return 8

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x1b2a79973e8bf53d7b53acb71299cb57
