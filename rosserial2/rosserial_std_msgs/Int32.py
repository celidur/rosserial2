from ..fuction import *


class Int32:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_int32(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_int32(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0xda5909fbe378aeaf85e547e830cc1bb7
