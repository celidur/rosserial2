#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import array
import multiprocessing
import queue
import importlib
import struct
import sys
import threading
import time
from serial import Serial, SerialException, SerialTimeoutException
import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.qos_event

_node = None
_logger = None
_on_shutdown = None

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"


def getRosType(typeName, message_type):
    if typeName == "Int8":
        return typeRosInt8(message_type)
    elif typeName == "Int16":
        return typeRosInt16(message_type)
    elif typeName == "Int32":
        return typeRosInt32(message_type)
    elif typeName == "Int64":
        return typeRosInt64(message_type)
    elif typeName == "UInt8":
        return typeRosUInt8(message_type)
    elif typeName == "UInt16":
        return typeRosUInt16(message_type)
    elif typeName == "UInt32":
        return typeRosUInt32(message_type)
    elif typeName == "UInt64":
        return typeRosUInt64(message_type)
    elif typeName == "Float32":
        return typeRosFloat32(message_type)
    elif typeName == "Float64":
        return typeRosFloat64(message_type)
    elif typeName == "String":
        return typeRosString(message_type)
    elif typeName == "Bool":
        return typeRosBool(message_type)
    else:
        return typeRos(message_type, "", {})


class typeRos:
    def __init__(self, message_type, md5sum, data):
        self._md5sum = md5sum
        self._type = message_type
        self.data = data

    def serialize(self, message):
        return None

    def deserialize(self, data, message):
        return None


class typeRosInt8(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "27ffa0c9c4b8fb8492252bcad9e5c57b", {"data": 0})

    def serialize(self, message):
        return struct.pack('b', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('b', data)[0]
        return message


class typeRosInt16(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "8524586e34fbd7cb1c08c5f5f1ca0e57", {"data": 0})

    def serialize(self, message):
        return struct.pack('h', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('h', data)[0]
        return message


class typeRosInt32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "da5909fbe378aeaf85e547e830cc1bb7", {"data": 0})

    def serialize(self, message):
        return struct.pack('i', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('i', data)[0]
        return message


class typeRosInt64(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "34add168574510e6e17f5d23ecc077ef", {"data": 0})

    def serialize(self, message):
        return struct.pack('q', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('q', data)[0]
        return message


class typeRosUInt8(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "7c8164229e7d2c17eb95e9231617fdee", {"data": 0})

    def serialize(self, message):
        return struct.pack('B', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('B', data)[0]
        return message


class typeRosUInt16(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "1df79edf208b629fe6b81923a544552d", {"data": 0})

    def serialize(self, message):
        return struct.pack('H', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('H', data)[0]
        return message


class typeRosUInt32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "304a39449588c7f8ce2df6e8001c5fce", {"data": 0})

    def serialize(self, message):
        return struct.pack('I', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('I', data)[0]
        return message


class typeRosUInt64(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "1b2a79973e8bf53d7b53acb71299cb57", {"data": 0})

    def serialize(self, message):
        return struct.pack('Q', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('Q', data)[0]
        return message


class typeRosFloat32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "73fcbf46b49191e672908e50842a83d4", {"data": 0})

    def serialize(self, message):
        return struct.pack('f', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('f', data)[0]
        return message


class typeRosFloat64(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "fdb28210bfa9d7c91146260178d9a584", {"data": 0})

    def serialize(self, message):
        return struct.pack('d', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('d', data)[0]
        return message


class typeRosString(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "992ce8a1687cec8c8bd883ec73ca41d1", {"data": ""})

    def serialize(self, message):
        length = len(message.data)
        return struct.pack('I', length) + message.data.encode('utf-8')

    def deserialize(self, data, message):
        message.data = data[4:].decode('utf-8')
        return message


class typeRosBool(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "8b94c1b53db61fb6aed406028ad6332a", {"data": False})

    def serialize(self, message):
        return struct.pack('?', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('?', data)[0]
        return message



def set_on_shutdown(on_shutdown):
    global _on_shutdown
    _on_shutdown = on_shutdown

def _thread_spin_target():
    global _node, _on_shutdown
    rclpy.spin(_node)
    if _on_shutdown:
        _on_shutdown()


def import_module(module_name, module_dir=None):
    """
        desc
    """
    if module_dir:
        sys.path.insert(0, module_dir)
    try:
        module = importlib.import_module(module_name)
    except:
        raise "File not found"
    finally:
        if module_dir:
            sys.path.pop(0)
        return module


class TopicInfo:
    ID_PUBLISHER = 0
    ID_SUBSCRIBER = 1
    ID_SERVICE_SERVER = 2
    ID_SERVICE_CLIENT = 4
    ID_PARAMETER_REQUEST = 6
    ID_LOG = 7
    ID_TIME = 10
    ID_TX_STOP = 11

    def __init__(self):
        self.topic_id = 0
        self.topic_name = ""
        self.message_type = ""
        self.md5sum = ""
        self.buffer_size = 0

    def serialize(self):
        _logger.warning('it is not implemented')

    def deserialize(self, data):
        offset = 0
        self.topic_id = struct.unpack('h', data[offset:offset + 2])[0]
        offset += 2
        length_topic_name = struct.unpack('i', data[offset:offset + 4])[0]
        offset += 4
        self.topic_name = data[offset:offset + length_topic_name].decode('utf-8')
        offset += length_topic_name
        length_message_type = struct.unpack('i', data[offset:offset + 4])[0]
        offset += 4
        self.message_type = data[offset:offset + length_message_type].decode('utf-8')
        offset += length_message_type
        length_md5sum = struct.unpack('i', data[offset:offset + 4])[0]
        offset += 4
        self.md5sum = data[offset:offset + length_md5sum].decode('utf-8')
        offset += length_md5sum
        self.buffer_size = struct.unpack('i', data[offset:offset + 4])[0]
        offset += 4
        return offset


def load_pkg_module(package, directory):
    # check if its in the python path
    path = sys.path
    try:
        importlib.import_module(package)
    except ImportError:
        pass
    try:
        m = __import__(package + '.' + directory)
    except ImportError:
        _logger.error("Cannot import package : %s" % package)
        _logger.error("sys.path was " + str(path))
        return None
    return m


def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

# TODO: implement
# def load_service(package,service):
#     s = load_pkg_module(package, 'srv')
#     s = getattr(s, 'srv')
#     srv = getattr(s, service)
#     mreq = getattr(s, service+"Request")
#     mres = getattr(s, service+"Response")
#     return srv,mreq,mres

class Publisher:
    """
        Publisher forwards messages from the serial device to ROS.
    """

    def __init__(self, topic_info):
        """ Create a new publisher. """
        self.topic = topic_info.topic_name

        # find message type
        package, message = topic_info.message_type.split('/')
        self.manage = getRosType(message, topic_info.message_type)
        self.message = load_message(package, message)
        if self.manage._md5sum == topic_info.md5sum:
            self.publisher = _node.create_publisher(
                self.message,
                self.topic,
                rclpy.qos.QoSProfile(depth=10, history=rclpy.qos.HistoryPolicy.KEEP_LAST)
            )
        else:
            _logger.warning('This type is not implemented : ' + topic_info.message_type)

    def handlePacket(self, data):
        """ Forward message to ROS network. """
        t = self.manage.deserialize(data, self.message())
        if t is not None:
            self.publisher.publish(t)


class Subscriber:
    """
        Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent

        # find message type
        package, message = topic_info.message_type.split('/')
        self.manage = getRosType(message, topic_info.message_type)
        self.message = load_message(package, message)
        if self.manage._md5sum == topic_info.md5sum:
            self.subscriber = _node.create_subscription(self.message, self.topic, self.callback, 10,
                                                        event_callbacks=rclpy.qos_event.SubscriptionEventCallbacks())
        else:
            _logger.warning('This type is not implemented : ' + topic_info.message_type)

    def callback(self, msg):
        """ Forward message to serial device. """
        t = self.manage.serialize(msg)
        if t is not None:
            self.parent.send(self.id, t)

    def unregister(self):
        _logger.warning("Removing subscriber: ", self.topic)
        self.subscriber.unregister()

# TODO: implement service client
# class ServiceServer:
#     """
#         ServiceServer responds to requests from ROS.
#     """
#
#     def __init__(self, topic_info, parent):
#         self.topic = topic_info.topic_name
#         self.parent = parent
#
#         # find message type
#         package, service = topic_info.message_type.split('/')
#         s = load_pkg_module(package, 'srv')
#         s = getattr(s, 'srv')
#         self.mreq = getattr(s, service + "Request")
#         self.mres = getattr(s, service + "Response")
#         srv = getattr(s, service)
#         self.service = rospy.Service(self.topic, srv, self.callback)
#
#         # response message
#         self.data = None
#
#     def unregister(self):
#         print("Removing service: %s", self.topic)
#         self.service.shutdown()
#
#     def callback(self, req):
#         """ Forward request to serial device. """
#         data_buffer = io.BytesIO()
#         req.serialize(data_buffer)
#         self.response = None
#         self.parent.send(self.id, data_buffer.getvalue())
#         while self.response is None:
#             pass
#         return self.response
#
#     def handlePacket(self, data):
#         """ Forward response to ROS network. """
#         r = self.mres()
#         r.deserialize(data)
#         self.response = r
#

# TODO: implement service client
# class ServiceClient:
#     """
#         ServiceServer responds to requests from ROS.
#     """
#
#     def __init__(self, topic_info, parent):
#         self.topic = topic_info.topic_name
#         self.parent = parent
#
#         # find message type
#         package, service = topic_info.message_type.split('/')
#         s = load_pkg_module(package, 'srv')
#         s = getattr(s, 'srv')
#         self.mreq = getattr(s, service + "Request")
#         self.mres = getattr(s, service + "Response")
#         srv = getattr(s, service)
#         print("Starting service client, waiting for service '" + self.topic + "'")
#         rospy.wait_for_service(self.topic)
#         self.proxy = rospy.ServiceProxy(self.topic, srv)
#
#     def handlePacket(self, data):
#         """ Forward request to ROS network. """
#         req = self.mreq()
#         req.deserialize(data)
#         # call service proxy
#         resp = self.proxy(req)
#         # serialize and publish
#         data_buffer = io.BytesIO()
#         resp.serialize(data_buffer)
#         self.parent.send(self.id, data_buffer.getvalue())


class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """
    header = b'\xff'

    # hydro introduces protocol ver2 which must match node_handle.h
    # The protocol version is sent as the 2nd sync byte emitted by each end
    protocol_ver1 = b'\xff'
    protocol_ver2 = b'\xfe'
    protocol_ver = protocol_ver2

    def __init__(self, port=None, baud=500000, timeout=5.0):
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        self.read_lock = threading.RLock()

        self.write_lock = threading.RLock()
        self.write_queue = queue.Queue()
        self.write_thread = None

        self.lastsync = time.time()
        self.lastsync_lost = time.time()
        self.lastsync_success = time.time()
        self.last_read = time.time()
        self.last_write = time.time()
        self.timeout = timeout
        self.synced = False

        self.publishers = dict()  # id:Publishers
        self.subscribers = dict()  # topic:Subscriber
        self.services = dict()  # topic:Service

        def shutdown():
            self.txStopRequest()
            _logger.info('shutdown hook activated')

        set_on_shutdown(shutdown)

        self.pub_diagnostics = _node.create_publisher(load_message("diagnostic_msgs","DiagnosticArray"), '/diagnostics',
                                                      rclpy.qos.QoSProfile(depth=10,
                                                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST))

        if port is None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            # assume its a filelike object
            self.port = port
        else:
            # open a specific port
            while rclpy.ok():
                try:
                    self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
                    break
                except SerialException as e:
                    _logger.error("Error opening serial: %s"% e)
                    time.sleep(3)

        if not rclpy.ok():
            return

        # time.sleep(0.1)  # Wait for ready (patch for Uno)

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER + TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER + TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT + TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT + TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        time.sleep(2.0)
        self.requestTopics()
        self.lastsync = time.time()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        _logger.info('Requesting topics...')

        # request topic sync
        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x0b\x00\xf4")
        _logger.info("Sending tx stop request")

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = time.time()
                    result.extend(received)
                    bytes_remaining -= len(received)

            if bytes_remaining != 0:
                raise IOError(
                    "Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def run(self):
        """ Forward recieved messages to appropriate publisher. """

        # Launch write thread.
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self.processWriteQueue)
            self.write_thread.daemon = True
            self.write_thread.start()

        # Handle reading.
        read_step = None
        while rclpy.ok():
            if (time.time() - self.lastsync) > (self.timeout * 3):
                if self.synced:
                    _logger.error("Lost sync with device, restarting...")
                else:
                    _logger.error(
                        "Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = time.time()
                # self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_NO_SYNC)
                self.requestTopics()
                self.lastsync = time.time()

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:
                        time.sleep(0.001)
                        continue

                # Find sync flag.
                flag = [0, 0]
                read_step = 'syncflag'
                flag[0] = self.tryRead(1)
                if (flag[0] != self.header):
                    continue

                # Find protocol version.
                read_step = 'protocol'
                flag[1] = self.tryRead(1)
                if flag[1] != self.protocol_ver:
                    # self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_MISMATCHED_PROTOCOL)
                    _logger.error(
                        "Mismatched protocol version in packet (%s): lost sync or rosserial_python is from different ros release than the rosserial client" % repr(
                            flag[1]))
                    protocol_ver_msgs = {
                        self.protocol_ver1: 'Rev 0 (rosserial 0.4 and earlier)',
                        self.protocol_ver2: 'Rev 1 (rosserial 0.5+)',
                        b'\xfd': 'Some future rosserial version'
                    }
                    if flag[1] in protocol_ver_msgs:
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    _logger.error("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                # Read message length, checksum (3 bytes)
                read_step = 'message length'
                msg_len_bytes = self.tryRead(3)
                msg_length, _ = struct.unpack("<hB", msg_len_bytes)

                # Validate message length checksum.
                if sum(array.array("B", msg_len_bytes)) % 256 != 255:
                    _logger.error("Wrong checksum for msg length, length %d, dropping message." % (msg_length))
                    continue

                # Read topic id (2 bytes)
                read_step = 'topic id'
                topic_id_header = self.tryRead(2)
                topic_id, = struct.unpack("<H", topic_id_header)

                # Read serialized message data.
                read_step = 'data'
                try:
                    msg = self.tryRead(msg_length)
                except IOError:
                    # self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                    _logger.error("Packet Failed :  Failed to read msg data")
                    _logger.error("expected msg length is %d"% msg_length)
                    raise

                # Reada checksum for topic id and msg
                read_step = 'data checksum'
                chk = self.tryRead(1)
                checksum = sum(array.array('B', topic_id_header + msg + chk))

                # Validate checksum.
                if checksum % 256 == 255:
                    self.synced = True
                    self.lastsync_success = time.time()
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        _logger.warning("Tried to publish before configured, topic id %d" % topic_id)
                        self.requestTopics()
                    time.sleep(0.001)
                else:
                    _logger.error("wrong checksum for topic id and msg")

            except IOError as exc:
                _logger.error('Last read step: %s' % read_step)
                _logger.error('Run loop error: %s' % exc)
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()
        self.write_thread.join()

    def setPublishSize(self, size):
        if self.buffer_out < 0:
            self.buffer_out = size
            _logger.info("Note: publish buffer size is %d bytes" % self.buffer_out)

    def setSubscribeSize(self, size):
        if self.buffer_in < 0:
            self.buffer_in = size
            _logger.info("Note: subscribe buffer size is %d bytes" % self.buffer_in)

    def setupPublisher(self, data: bytes):
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            _logger.info("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type))
        except Exception as e:
            _logger.error("Creation of publisher failed: %s", e)

    def setupSubscriber(self, data: bytes):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in list(self.subscribers.keys()):
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                _logger.info("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type))
            elif msg.message_type != self.subscribers[msg.topic_name].message._type:
                old_message_type = self.subscribers[msg.topic_name].message._type
                self.subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                _logger.info("Change the message type of subscriber on %s from [%s] to [%s]" % (
                    msg.topic_name, old_message_type, msg.message_type))
        except Exception as e:
            _logger.error("Creation of subscriber failed: %s", e)

    # TODO: implement
    def setupServiceServerPublisher(self, data: bytes):
        """ Register a new service server. """
        _logger.error("setupServiceServerPublisher")
        # try:
        #     msg = TopicInfo()
        #     msg.deserialize(data)
        #     self.setPublishSize(msg.buffer_size)
        #     try:
        #         srv = self.services[msg.topic_name]
        #     except KeyError:
        #         srv = ServiceServer(msg, self)
        #         print("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type))
        #         self.services[msg.topic_name] = srv
        #     if srv.mres._md5sum == msg.md5sum:
        #         self.callbacks[msg.topic_id] = srv.handlePacket
        #     else:
        #         raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        # except Exception as e:
        #     print("Creation of service server failed: %s", e)

    # TODO: implement
    def setupServiceServerSubscriber(self, data: bytes):
        """ Register a new service server. """
        _logger.error("setupServiceServerSubscriber")
        # try:
        #     msg = TopicInfo()
        #     msg.deserialize(data)
        #     self.setSubscribeSize(msg.buffer_size)
        #     try:
        #         srv = self.services[msg.topic_name]
        #     except KeyError:
        #         srv = ServiceServer(msg, self)
        #         print("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type))
        #         self.services[msg.topic_name] = srv
        #     if srv.mreq._md5sum == msg.md5sum:
        #         srv.id = msg.topic_id
        #     else:
        #         raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        # except Exception as e:
        #     print("Creation of service server failed: %s", e)

    # TODO: implement
    def setupServiceClientPublisher(self, data: bytes):
        """ Register a new service client. """
        _logger.error("setupServiceClientPublisher")
        # try:
        #     msg = TopicInfo()
        #     msg.deserialize(data)
        #     self.setPublishSize(msg.buffer_size)
        #     try:
        #         srv = self.services[msg.topic_name]
        #     except KeyError:
        #         srv = ServiceClient(msg, self)
        #         print("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type))
        #         self.services[msg.topic_name] = srv
        #     if srv.mreq._md5sum == msg.md5sum:
        #         self.callbacks[msg.topic_id] = srv.handlePacket
        #     else:
        #         raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        # except Exception as e:
        #     print("Creation of service client failed: %s", e)

    # TODO: implement
    def setupServiceClientSubscriber(self, data: bytes):
        """ Register a new service client. """
        _logger.error("setupServiceClientSubscriber")
        # try:
        #     msg = TopicInfo()
        #     msg.deserialize(data)
        #     self.setSubscribeSize(msg.buffer_size)
        #     try:
        #         srv = self.services[msg.topic_name]
        #     except KeyError:
        #         srv = ServiceClient(msg, self)
        #         print("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type))
        #         self.services[msg.topic_name] = srv
        #     if srv.mres._md5sum == msg.md5sum:
        #         srv.id = msg.topic_id
        #     else:
        #         raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        # except Exception as e:
        #     print("Creation of service client failed: %s", e)

    def handleTimeRequest(self, data: bytes):
        """ Respond to device with system time. """
        a = time.time_ns()
        res = (a // (10 ** 9)).to_bytes(4, byteorder='little') + (a % (10 ** 9)).to_bytes(4, byteorder='little')
        self.send(TopicInfo.ID_TIME, res)
        self.lastsync = time.time()

    # TODO: implement
    def handleParameterRequest(self, data: bytes):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        _logger.error("handleParameterRequest")
        # req = RequestParamRequest()
        # req.deserialize(data)
        # resp = RequestParamResponse()
        # try:
        #     param = rospy.get_param(req.name)
        # except KeyError:
        #     print("Parameter %s does not exist" % req.name)
        #     return
        #
        # if param is None:
        #     print("Parameter %s does not exist" % req.name)
        #     return
        #
        # if isinstance(param, dict):
        #     print("Cannot send param %s because it is a dictionary" % req.name)
        #     return
        # if not isinstance(param, list):
        #     param = [param]
        # # check to make sure that all parameters in list are same type
        # t = type(param[0])
        # for p in param:
        #     if t != type(p):
        #         print('All Paramers in the list %s must be of the same type' % req.name)
        #         return
        # if t == int or t == bool:
        #     resp.ints = param
        # if t == float:
        #     resp.floats = param
        # if t == str:
        #     resp.strings = param
        # data_buffer = io.BytesIO()
        # resp.serialize(data_buffer)
        # self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())

    # TODO: implement
    def handleLoggingRequest(self, data: bytes):
        """ Forward logging information from serial device into ROS. """
        _logger.error("handleLoggingRequest :" + str(data))
        # msg = Log()
        # msg.deserialize(data)
        # if msg.level == Log.ROSDEBUG:
        #     rospy.logdebug(msg.msg)
        # elif msg.level == Log.INFO:
        #     print(msg.msg)
        # elif msg.level == Log.WARN:
        #     print(msg.msg)
        # elif msg.level == Log.ERROR:
        #     print(msg.msg)
        # elif msg.level == Log.FATAL:
        #     rospy.logfatal(msg.msg)

    def send(self, topic, msg):
        """
        Queues data to be written to the serial port.
        """
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/rosserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = time.time()

    def _send(self, topic, msg_bytes):
        """
        Send a message on a particular topic to the device.
        """
        length = len(msg_bytes)
        if self.buffer_in > 0 and length > self.buffer_in:
            _logger.error("Message from ROS network dropped: message larger than buffer.\n%s" % msg_bytes)
            return -1
        else:
            # frame : header (1b) + version (1b) + msg_len(2b) + msg_len_chk(1b) + topic_id(2b) + msg(nb) + msg_topic_id_chk(1b)
            length_bytes = struct.pack('<h', length)
            length_checksum = 255 - (sum(array.array('B', length_bytes)) % 256)
            length_checksum_bytes = struct.pack('B', length_checksum)

            topic_bytes = struct.pack('<h', topic)
            msg_checksum = 255 - (sum(array.array('B', topic_bytes + msg_bytes)) % 256)
            msg_checksum_bytes = struct.pack('B', msg_checksum)

            self._write(
                self.header + self.protocol_ver + length_bytes + length_checksum_bytes + topic_bytes + msg_bytes + msg_checksum_bytes)
            return length

    def processWriteQueue(self):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """
        while rclpy.ok():
            if self.write_queue.empty():
                time.sleep(0.01)
            else:
                data = self.write_queue.get()
                while rclpy.ok():
                    try:
                        if isinstance(data, tuple):
                            topic, msg = data
                            self._send(topic, msg)
                        elif isinstance(data, bytes):
                            self._write(data)
                        else:
                            _logger.error("Trying to write invalid data type: %s" % type(data))
                        break
                    except SerialTimeoutException as exc:
                        _logger.error('Write timeout: %s' % exc)
                        time.sleep(1)
                    except RuntimeError as exc:
                        _logger.error('Write thread exception: %s' % exc)
                        break

    # TODO: implement
    def sendDiagnostics(self, level, msg_text):
        _logger.error("\033[91m%s\033[0m" + "sendDiagnostics")
        # msg = diagnostic_msgs.msg.DiagnosticArray()
        # status = diagnostic_msgs.msg.DiagnosticStatus()
        # status.name = "rosserial_python"
        # msg.header.stamp = time.time()
        # msg.status.append(status)
        #
        # status.message = msg_text
        # status.level = level
        #
        # status.values.append(diagnostic_msgs.msg.KeyValue())
        # status.values[0].key = "last sync"
        # if self.lastsync.to_sec() > 0:
        #     status.values[0].value = time.ctime(self.lastsync.to_sec())
        # else:
        #     status.values[0].value = "never"
        #
        # status.values.append(diagnostic_msgs.msg.KeyValue())
        # status.values[1].key = "last sync lost"
        # status.values[1].value = time.ctime(self.lastsync_lost.to_sec())
        #
        # self.pub_diagnostics.publish(msg)


def main():
    global _node, _logger
    rclpy.init()
    _node = rclpy.create_node(
        "serial_node",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    _logger = _node.get_logger()
    rclpy.logging.set_logger_level(_logger.name, rclpy.logging.LoggingSeverity.INFO)
    _thread_spin = threading.Thread(target=_thread_spin_target, daemon=True)
    _thread_spin.start()
    _logger.info("ROS Serial Python Node")

    port = '/dev/ttyUSB0'
    if not _node.has_parameter("port"):
        _node.declare_parameter("port", port)
    port = _node.get_parameter("port")._value

    baud = '500_000'
    if not _node.has_parameter("baud"):
        _node.declare_parameter("baud", baud)
    baud = int(_node.get_parameter("baud")._value)

    while rclpy.ok():
        _logger.info("Connecting to %s at %d baud" % (port, baud))
        try:
            client = SerialClient(port, baud)
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            time.sleep(1.0)
            continue
        except OSError:
            time.sleep(1.0)
            continue
        except:
            _logger.warning("Unexpected Error: %s", sys.exc_info()[0])
            client.port.close()
            time.sleep(1.0)
            continue


if "__main__" == __name__:
    main()