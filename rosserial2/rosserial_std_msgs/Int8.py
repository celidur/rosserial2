from ..fuction import *


class Int8:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_int8(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_int8(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x27ffa0c9c4b8fb8492252bcad9e5c57b
