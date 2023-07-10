import struct


class Float32:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.data = message.data
        return struct.pack('f', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('f', data[0:4])[0]
        return 4

    def __dict__(self):
        return {'data': self.data}

    def __set__(self, instance, value):
        self.data = value.data

    def __hash__(self):
        return 0x73fcbf46b49191e672908e50842a83d4
