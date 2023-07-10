from typing import Union

import std_msgs


class Char:
    def __init__(self):
        self.data = std_msgs.Int8()

    def serialize(self, message=None):
        if message is not None:
            self.data.data = message.data
        return self.data.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.data.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'data': self.data.data}

    def __set__(self, instance, value):
        self.data = value

    def __hash__(self):
        return 0x1bf77f25acecdedba0e224b162199717
