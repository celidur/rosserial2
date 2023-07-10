import std_msgs


class Header:
    def __init__(self):
        self.seq = std_msgs.UInt32()
        self.stamp = std_msgs.Time()
        self.frame_id = std_msgs.Char

    def serialize(self, message=None):
        if message is None:
            message = self
        self.seq = message.seq
        self.stamp = message.stamp
        self.frame_id = message.frame_id
        return self.seq

    def deserialize(self, data):
        offset = 0
        offset += self.seq.deserialize(data[offset:])
        offset += self.stamp.deserialize(data[offset:])
        offset += self.frame_id.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'stamp': self.stamp.get_data(), 'frame_id': self.frame_id.data}

    def __set__(self, instance, value):
        self.stamp = value.stamp
        self.frame_id = value.frame_id

    def __hash__(self):
        return 0x1bf77f25acecdedba0e224b162199717
