import struct


class Int16:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('h', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('h', data[0:2])[0]
        return 2

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x8524586e34fbd7cb1c08c5f5f1ca0e57
