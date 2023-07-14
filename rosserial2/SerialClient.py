import struct
import threading
from serial import SerialException, Serial, SerialTimeoutException
import time
import queue
import array
import rclpy
from .LoadMsg import load_message
import rosserial2 as ros2
from .Subscriber import Subscriber
from .Publisher import Publisher
from .rosserial_msgs import TopicInfo, Log
import rosserial2.rosserial_std_msgs as std_msgs

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"


# TODO: implement
# def load_service(package,service):
#     s = load_pkg_module(package, 'srv')
#     s = getattr(s, 'srv')
#     srv = getattr(s, service)
#     mreq = getattr(s, service+"Request")
#     mres = getattr(s, service+"Response")
#     return srv,mreq,mres

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
            ros2._logger.info('shutdown hook activated')

        ros2._on_shutdown = shutdown

        self.pub_diagnostics = ros2._node.create_publisher(load_message("diagnostic_msgs", "DiagnosticArray"),
                                                           '/diagnostics',
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
                    ros2._logger.error("Error opening serial: %s" % e)
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
        ros2._logger.info('Requesting topics...')

        # request topic sync
        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x0b\x00\xf4")
        ros2._logger.info("Sending tx stop request")

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
                    ros2._logger.error("Lost sync with device, restarting...")
                else:
                    ros2._logger.error(
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
                    ros2._logger.error(
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
                    ros2._logger.error("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                # Read message length, checksum (3 bytes)
                read_step = 'message length'
                msg_len_bytes = self.tryRead(3)
                msg_length, _ = struct.unpack("<hB", msg_len_bytes)

                # Validate message length checksum.
                if sum(array.array("B", msg_len_bytes)) % 256 != 255:
                    ros2._logger.error("Wrong checksum for msg length, length %d, dropping message." % (msg_length))
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
                    ros2._logger.error("Packet Failed :  Failed to read msg data")
                    ros2._logger.error("expected msg length is %d" % msg_length)
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
                        ros2._logger.warning("Tried to publish before configured, topic id %d" % topic_id)
                        self.requestTopics()
                    except Exception as e:
                        ros2._logger.error("Error during deserialization of topic id %d: %s" % (topic_id, e))
                    time.sleep(0.001)
                else:
                    ros2._logger.error("wrong checksum for topic id and msg")

            except IOError as exc:
                ros2._logger.error('Last read step: %s' % read_step)
                ros2._logger.error('Run loop error: %s' % exc)
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
            ros2._logger.info("Note: publish buffer size is %d bytes" % self.buffer_out)

    def setSubscribeSize(self, size):
        if self.buffer_in < 0:
            self.buffer_in = size
            ros2._logger.info("Note: subscribe buffer size is %d bytes" % self.buffer_in)

    def setupPublisher(self, data: bytes):
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            ros2._logger.info("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type))
        except Exception as e:
            ros2._logger.error("Creation of publisher failed: %s" % e)

    def setupSubscriber(self, data: bytes):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in list(self.subscribers.keys()):
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                ros2._logger.info("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type))
            elif msg.message_type != self.subscribers[msg.topic_name].message.type():
                old_message_type = self.subscribers[msg.topic_name].manage._type
                self.subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                ros2._logger.info("Change the message type of subscriber on %s from [%s] to [%s]" % (
                    msg.topic_name, old_message_type, msg.message_type))
        except Exception as e:
            ros2._logger.error("Creation of subscriber failed: %s", e)

    # TODO: implement
    def setupServiceServerPublisher(self, data: bytes):
        """ Register a new service server. """
        ros2._logger.error("setupServiceServerPublisher")
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
        ros2._logger.error("setupServiceServerSubscriber")
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
        ros2._logger.error("setupServiceClientPublisher")
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
        ros2._logger.error("setupServiceClientSubscriber")
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
        self.send(TopicInfo.ID_TIME, std_msgs.Time().serialize(ros2._time().to_msg()))
        self.lastsync = time.time()

    # TODO: implement
    def handleParameterRequest(self, data: bytes):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        ros2._logger.error("handleParameterRequest")
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

    def handleLoggingRequest(self, data: bytes):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if msg.level == Log.ROSDEBUG:
            ros2._logger.debug(msg.msg)
        elif msg.level == Log.INFO:
            ros2._logger.info(msg.msg)
        elif msg.level == Log.WARN:
            ros2._logger.warning(msg.msg)
        elif msg.level == Log.ERROR:
            ros2._logger.error(msg.msg)
        elif msg.level == Log.FATAL:
            ros2._logger.fatal(msg.msg)

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
            ros2._logger.error("Message from ROS network dropped: message larger than buffer.\n%s" % msg_bytes)
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
                            ros2._logger.error("Trying to write invalid data type: %s" % type(data))
                        break
                    except SerialTimeoutException as exc:
                        ros2._logger.error('Write timeout: %s' % exc)
                        time.sleep(1)
                    except RuntimeError as exc:
                        ros2._logger.error('Write thread exception: %s' % exc)
                        break

    # TODO: implement
    def sendDiagnostics(self, level, msg_text):
        ros2._logger.error("sendDiagnostics")
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
