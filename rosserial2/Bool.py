import struct

class Bool:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('?', self.data)

    def deserialize(self, data):
        self.data = struct.unpack('?', data[0:1])[0]
        return 1

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x8b94c1b53db61fb6aed406028ad6332a
