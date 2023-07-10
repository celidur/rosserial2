import struct


class Int8:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('b', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('b', data[0:1])[0]
        return 1

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x27ffa0c9c4b8fb8492252bcad9e5c57b
