import struct


class Int16:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('h', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('h', data[0:2])[0]
        return 2

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x8524586e34fbd7cb1c08c5f5f1ca0e57
