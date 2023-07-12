from .. import rosserial_std_msgs as std_msgs

class Vector3:
    def __init__(self):
        self.x = std_msgs.Float64()
        self.y = std_msgs.Float64()
        self.z = std_msgs.Float64()

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return self.x.serialize() + self.y.serialize() + self.z.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.x.deserialize(data[offset:])
        offset += self.y.deserialize(data[offset:])
        offset += self.z.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {"x": self.x.data, "y": self.y.data, "z": self.z.data}

    def set(self, value):
        self.x.data = value.x
        self.y.data = value.y
        self.z.data = value.z

    def __hash__(self):
        return 0x4a842b65f413084dc2b10fb484ea7f17

