import struct


class Int64:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('q', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('q', data[0:8])[0]
        return 8

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x34add168574510e6e17f5d23ecc077ef
