from ..fuction import *


class String:
    def __init__(self):
        self.data: str = ""

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_string(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_string(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x992ce8a1687cec8c8bd883ec73ca41d1
