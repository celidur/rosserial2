import struct


class UInt16:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('H', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('H', data[0:2])[0]
        return 2

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x1df79edf208b629fe6b81923a544552d
