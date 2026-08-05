import time as _time
from typing import Any, Callable, Optional

import rclpy
from builtin_interfaces.msg import Time as _TimeMsg
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from std_msgs.msg import Header


def _to_sec(self):
    return float(self.sec) + float(self.nanosec) / 1e9


if not hasattr(_TimeMsg, "to_sec"):
    _TimeMsg.to_sec = _to_sec

_node = None
_subscriptions = []
_publishers = []


class ROSInterruptException(Exception):
    pass


class Time:
    def __new__(cls, *args, **kwargs):
        return _TimeMsg(*args, **kwargs)

    @staticmethod
    def now():
        return _ensure_node().get_clock().now().to_msg()


def _normalize_node_name(name: str) -> str:
    normalized = name.strip("/").replace("/", "_").replace("-", "_")
    return normalized or "fsdp_node"


def _ensure_node(name: str = "fsdp_node"):
    global _node
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = rclpy.create_node(_normalize_node_name(name), automatically_declare_parameters_from_overrides=True)
    return _node


def init_node(name: str, anonymous: bool = False):
    suffix = f"_{int(_time.time() * 1000)}" if anonymous else ""
    return _ensure_node(f"{name}{suffix}")


def get_node():
    return _ensure_node()


def is_shutdown() -> bool:
    return not rclpy.ok()


def spin():
    try:
        rclpy.spin(_ensure_node())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def loginfo(msg: Any):
    _ensure_node().get_logger().info(str(msg))


def logwarn(msg: Any):
    _ensure_node().get_logger().warning(str(msg))


def logerr(msg: Any):
    _ensure_node().get_logger().error(str(msg))


# ROS1 의 rospy.log*_throttle(period, msg) 시그니처를 유지한다.
def loginfo_throttle(period: float, msg: Any):
    _ensure_node().get_logger().info(str(msg), throttle_duration_sec=float(period))


def logwarn_throttle(period: float, msg: Any):
    _ensure_node().get_logger().warning(str(msg), throttle_duration_sec=float(period))


def logerr_throttle(period: float, msg: Any):
    _ensure_node().get_logger().error(str(msg), throttle_duration_sec=float(period))


def _param_name(name: str) -> str:
    return name.strip("/").replace("/", ".").lstrip("~")


def get_param(name: str, default: Any = None):
    node = _ensure_node()
    pname = _param_name(name)
    if not node.has_parameter(pname):
        node.declare_parameter(pname, default)
    return node.get_parameter(pname).value


def set_param(name: str, value: Any):
    node = _ensure_node()
    pname = _param_name(name)
    if not node.has_parameter(pname):
        node.declare_parameter(pname, value)
    else:
        node.set_parameters([Parameter(pname, value=value)])


class Rate:
    def __init__(self, hz: float):
        self.period = 1.0 / float(hz) if hz else 0.0

    def sleep(self):
        node = _ensure_node()
        rclpy.spin_once(node, timeout_sec=0.0)
        if self.period > 0:
            _time.sleep(self.period)


class Publisher:
    def __init__(self, topic: str, msg_type: Any, queue_size: int = 10, **kwargs):
        self._publisher = _ensure_node().create_publisher(msg_type, topic, queue_size)
        _publishers.append(self._publisher)

    def publish(self, msg: Any):
        self._publisher.publish(msg)


class Subscriber:
    def __init__(self, topic: str, msg_type: Any, callback: Callable, queue_size: int = 10, **kwargs):
        self._subscription = _ensure_node().create_subscription(msg_type, topic, callback, queue_size)
        _subscriptions.append(self._subscription)


def wait_for_message(topic: str, msg_type: Any, timeout: Optional[float] = None):
    node = _ensure_node()
    state = {"msg": None}

    def _cb(msg):
        state["msg"] = msg

    sub = node.create_subscription(msg_type, topic, _cb, 10)
    start = _time.monotonic()
    try:
        while rclpy.ok() and state["msg"] is None:
            if timeout is not None and _time.monotonic() - start > timeout:
                raise TimeoutError(f"Timed out waiting for {topic}")
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub)
    return state["msg"]


class ServiceProxy:
    def __init__(self, name: str, srv_type: Any = None):
        self.name = name
        self.srv_type = srv_type
        self._client = None
        if srv_type is not None:
            self._client = _ensure_node().create_client(srv_type, name)

    def __call__(self, *args, **kwargs):
        raise NotImplementedError(
            f"ROS1 ServiceProxy({self.name}) needs a ROS2 service/client port or direct frenet_conversion API replacement"
        )


class Config:
    pass


def declare_parameters(defaults: dict[str, Any]):
    node = _ensure_node()
    for name, value in defaults.items():
        pname = _param_name(name)
        if not node.has_parameter(pname):
            node.declare_parameter(pname, value, ParameterDescriptor())
    return {name: get_param(name, value) for name, value in defaults.items()}
