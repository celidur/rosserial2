from .Int8 import Int8


class Char:
    def __init__(self):
        self.data = Int8()

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return self.data.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.data.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'data': self.data.data}

    def set(self, value):
        self.data = value

    def __hash__(self):
        return 0x1bf77f25acecdedba0e224b162199717
