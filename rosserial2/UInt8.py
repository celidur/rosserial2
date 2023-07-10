import struct


class UInt8:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('B', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('B', data[0:1])[0]
        return 1

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x7c8164229e7d2c17eb95e9231617fdee
