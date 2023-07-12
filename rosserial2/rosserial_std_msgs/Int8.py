import struct


class Int8:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('b', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('b', data[0:1])[0]
        return 1

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x27ffa0c9c4b8fb8492252bcad9e5c57b
