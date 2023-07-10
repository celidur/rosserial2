from rosserial2.UInt32 import UInt32
import rosserial2 as ros2

class String:
    def __init__(self):
        self.len = UInt32()
        self.data = ""

    def serialize(self, message):
        return self.len.serialize(len(message.data)) + message.data.encode('utf-8')

    def deserialize(self, data):
        offset = 0
        offset = self.len.deserialize(data[offset:])
        size = self.len.data
        self.data = data[offset:offset + size].decode('utf-8')
        offset = offset + size
        return offset

    def get_data(self):
        return self.data

    def set_message_data(self, message):
        message.data = self.data
        return message

    def __hash__(self):
        return 0x992ce8a1687cec8c8bd883ec73ca41d1
