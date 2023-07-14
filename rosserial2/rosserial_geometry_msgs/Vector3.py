from ..fuction import *


class Vector3:
    def __init__(self):
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_float64(self.x) + serialization_float64(self.y) + serialization_float64(self.z)

    def deserialize(self, data, offset=0):
        offset, self.x = deserialization_float64(data, offset)
        offset, self.y = deserialization_float64(data, offset)
        offset, self.z = deserialization_float64(data, offset)
        return offset

    def __dict__(self):
        return {"x": self.x, "y": self.y, "z": self.z}

    def set(self, value):
        self.x = value.x
        self.y = value.y
        self.z = value.z

    def __hash__(self):
        return 0x4a842b65f413084dc2b10fb484ea7f17

