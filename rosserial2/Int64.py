import struct


class Int64:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('q', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('q', data[0:8])[0]
        return 8

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x34add168574510e6e17f5d23ecc077ef
