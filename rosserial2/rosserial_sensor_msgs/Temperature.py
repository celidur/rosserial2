from .. import rosserial_std_msgs as std_msgs


class Temperature:
    def __init__(self):
        self.header = std_msgs.Header()
        self.temperature = std_msgs.Float64()
        self.variance = std_msgs.Float64()

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return self.header.serialize() + self.temperature.serialize() + self.variance.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.header.deserialize(data[offset:])
        offset += self.temperature.deserialize(data[offset:])
        offset += self.variance.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {"header": self.header.__dict__(), "temperature": self.temperature.data,
                "variance": self.variance.data}

    def set(self, value):
        self.header.set(value.header)
        self.temperature.data = value.temperature
        self.variance.data = value.variance

    def __hash__(self):
        return 0xff71b307acdbe7c871a5a6d7ed359100
