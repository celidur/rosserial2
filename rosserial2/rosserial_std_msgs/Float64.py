import struct


class Float64:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('d', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('d', data[0:8])[0]
        return 8

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0xfdb28210bfa9d7c91146260178d9a584
