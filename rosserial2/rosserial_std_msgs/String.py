from .UInt32 import UInt32

class String:
    def __init__(self):
        self.len = UInt32()
        self.data = ""

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        self.len.data = len(self.data)
        return self.len.serialize() + self.data.encode('utf-8')

    def deserialize(self, data):
        offset = 0
        offset = self.len.deserialize(data[offset:])
        size = self.len.data
        self.data = data[offset:offset + size].decode('utf-8')
        offset = offset + size
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data
        self.len.data = len(self.data)

    def __hash__(self):
        return 0x992ce8a1687cec8c8bd883ec73ca41d1
