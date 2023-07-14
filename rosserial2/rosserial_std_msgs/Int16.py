from ..fuction import *


class Int16:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_int16(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_int16(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x8524586e34fbd7cb1c08c5f5f1ca0e57
