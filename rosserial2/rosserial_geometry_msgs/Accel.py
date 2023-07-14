from .Vector3 import Vector3


class Accel:
    def __init__(self):
        self.linear: Vector3 = Vector3()
        self.angular: Vector3 = Vector3()

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return self.linear.serialize() + self.angular.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.linear.deserialize(data[offset:])
        offset += self.angular.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'linear': self.linear.__dict__(), 'angular': self.angular.__dict__()}

    def set(self, value):
        self.linear = value.linear
        self.angular = value.angular

    def __hash__(self):
        return 0x9f195f881246fdfa2798d1d3eebca84a
