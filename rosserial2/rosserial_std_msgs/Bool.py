from ..fuction import *


class Bool:
    def __init__(self):
        self.data: bool = False

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_bool(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_bool(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x8b94c1b53db61fb6aed406028ad6332a
