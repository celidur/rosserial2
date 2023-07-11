from .. import rosserial_std_msgs as std_msgs

class Quaternion:
    def __init__(self):
        self.x = std_msgs.Float64()
        self.y = std_msgs.Float64()
        self.z = std_msgs.Float64()
        self.w = std_msgs.Float64()

    def serialize(self, message=None):
        if message is None:
            message = self
        return self.x.serialize() + self.y.serialize() + self.z.serialize() + self.w.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.x.deserialize(data[offset:])
        offset += self.y.deserialize(data[offset:])
        offset += self.z.deserialize(data[offset:])
        offset += self.w.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {"x": self.x.data, "y": self.y.data, "z": self.z.data, "w": self.w.data}

    def __set__(self, instance, value):
        self.x.data = value.x
        self.y.data = value.y
        self.z.data = value.z
        self.w.data = value.w

    def __hash__(self):
        return 0xa779879fadf0160734f906b8c19c7004

