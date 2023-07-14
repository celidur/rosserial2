from ..fuction import *


class Quaternion:
    def __init__(self):
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0
        self.w: float = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_float64(self.x) + serialization_float64(self.y) + serialization_float64(
            self.z) + serialization_float64(self.w)

    def deserialize(self, data, offset=0):
        offset, self.x = deserialization_float64(data, offset)
        offset, self.y = deserialization_float64(data, offset)
        offset, self.z = deserialization_float64(data, offset)
        offset, self.w = deserialization_float64(data, offset)
        return offset

    def __dict__(self):
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}

    def set(self, value):
        self.x = value.x
        self.y = value.y
        self.z = value.z
        self.w = value.w

    def __hash__(self):
        return 0xa779879fadf0160734f906b8c19c7004
