import struct
from typing import Union


class Bool:
    def __init__(self):
        self.data = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return struct.pack('?', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('?', data[0:1])[0]
        return 1

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x8b94c1b53db61fb6aed406028ad6332a
