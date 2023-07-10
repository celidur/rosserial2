import struct


class Float32:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('f', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('f', data[0:4])[0]
        return 4

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x73fcbf46b49191e672908e50842a83d4
