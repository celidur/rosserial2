import struct


class UInt16:
    def __init__(self):
        self.data = 0

    def serialize(self, message):
        self.data = message.data
        return struct.pack('H', self.data)

    def deserialize(self, data):  # return offset
        self.data = struct.unpack('H', data[0:2])[0]
        return 2

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x1df79edf208b629fe6b81923a544552d
