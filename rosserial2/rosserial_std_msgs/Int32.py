import struct


class Int32:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('i', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('i', data[0:4])[0]
        return 4

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0xda5909fbe378aeaf85e547e830cc1bb7
