from .NavSatStatus import NavSatStatus
from .. import rosserial_std_msgs as std_msgs

# TODO : implement Arrays first in std_msgs
class NavSatFix:
    def __init__(self):
        self.header = std_msgs.Header()
        self.status = NavSatStatus()
        self.latitude = std_msgs.Float64()
        self.longitude = std_msgs.Float64()
        self.altitude = std_msgs.Float64()
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = std_msgs.UInt8()