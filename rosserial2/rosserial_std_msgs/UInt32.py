import struct


class UInt32:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.data = message.data
        return struct.pack('I', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('I', data[0:4])[0]
        return 4

    def __dict__(self):
        return {'data': self.data}

    def __set__(self, instance, value):
        self.data = value.data

    def __hash__(self):
        return 0x304a39449588c7f8ce2df6e8001c5fce
