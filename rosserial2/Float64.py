import struct


class Float64:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('d', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('d', data[0:8])[0]
        return 8

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0xfdb28210bfa9d7c91146260178d9a584