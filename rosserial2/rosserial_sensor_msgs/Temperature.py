from .. import rosserial_std_msgs as std_msgs
from ..fuction import *


class Temperature:
    def __init__(self):
        self.header: std_msgs.Header = std_msgs.Header()
        self.temperature: int = 0
        self.variance: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return self.header.serialize() + serialization_float64(self.temperature) + serialization_float64(
            self.variance)

    def deserialize(self, data, offset=0):
        offset = self.header.deserialize(data, offset)
        offset, self.temperature = deserialization_float64(data, offset)
        offset, self.variance = deserialization_float64(data, offset)
        return offset

    def __dict__(self):
        return {"header": self.header.__dict__(), "temperature": self.temperature,
                "variance": self.variance}

    def set(self, value):
        self.header.set(value.header)
        self.temperature = value.temperature
        self.variance = value.variance

    def __hash__(self):
        return 0xff71b307acdbe7c871a5a6d7ed359100
