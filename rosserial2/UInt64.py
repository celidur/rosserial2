import struct


class UInt64:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('Q', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('Q', data[0:8])[0]
        return 8

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x1b2a79973e8bf53d7b53acb71299cb57
