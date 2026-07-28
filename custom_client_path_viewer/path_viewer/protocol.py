from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
from google.protobuf.message import DecodeError

DELTA_COUNT = 49
POINT_COUNT = DELTA_COUNT + 1
INTENTION_NAMES = {
    1: "攻击",
    2: "防守",
    3: "移动",
}


class ProtocolError(ValueError):
    """Raised when an MQTT payload violates RobotPathPlanInfo constraints."""


@dataclass(frozen=True, slots=True)
class PathPoint:
    index: int
    x_dm: int
    y_dm: int

    @property
    def x_m(self) -> float:
        return self.x_dm / 10.0

    @property
    def y_m(self) -> float:
        return self.y_dm / 10.0


@dataclass(frozen=True, slots=True)
class DecodedPath:
    intention: int
    sender_id: int
    points: tuple[PathPoint, ...]
    payload_size: int

    @property
    def intention_name(self) -> str:
        return INTENTION_NAMES.get(self.intention, f"未知({self.intention})")


def _build_message_class():
    file_descriptor = descriptor_pb2.FileDescriptorProto()
    file_descriptor.name = "robot_path_plan_info.proto"
    file_descriptor.syntax = "proto3"

    message = file_descriptor.message_type.add()
    message.name = "RobotPathPlanInfo"

    def add_field(
        name: str,
        number: int,
        field_type: int,
        label: int = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL,
        packed: bool = False,
    ) -> None:
        field = message.field.add()
        field.name = name
        field.number = number
        field.type = field_type
        field.label = label
        if packed:
            field.options.packed = True

    add_field("intention", 1, descriptor_pb2.FieldDescriptorProto.TYPE_UINT32)
    add_field("start_pos_x", 2, descriptor_pb2.FieldDescriptorProto.TYPE_UINT32)
    add_field("start_pos_y", 3, descriptor_pb2.FieldDescriptorProto.TYPE_UINT32)
    add_field(
        "offset_x",
        4,
        descriptor_pb2.FieldDescriptorProto.TYPE_INT32,
        descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED,
        packed=True,
    )
    add_field(
        "offset_y",
        5,
        descriptor_pb2.FieldDescriptorProto.TYPE_INT32,
        descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED,
        packed=True,
    )
    add_field("sender_id", 6, descriptor_pb2.FieldDescriptorProto.TYPE_UINT32)

    pool = descriptor_pool.DescriptorPool()
    pool.Add(file_descriptor)
    descriptor = pool.FindMessageTypeByName("RobotPathPlanInfo")
    return message_factory.GetMessageClass(descriptor)


RobotPathPlanInfoMessage = _build_message_class()


def _validate_offsets(name: str, values: Iterable[int]) -> tuple[int, ...]:
    result = tuple(int(value) for value in values)
    if len(result) != DELTA_COUNT:
        raise ProtocolError(
            f"{name} 长度为 {len(result)}，协议要求固定为 {DELTA_COUNT}"
        )
    for index, value in enumerate(result):
        if value < -128 or value > 127:
            raise ProtocolError(
                f"{name}[{index}]={value}，超出 int8 范围 [-128, 127]"
            )
    return result


def decode_robot_path_plan(payload: bytes) -> DecodedPath:
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("payload 必须是 bytes-like 对象")

    raw_payload = bytes(payload)
    message = RobotPathPlanInfoMessage()
    try:
        message.ParseFromString(raw_payload)
    except DecodeError as exc:
        raise ProtocolError(f"Protobuf 解析失败: {exc}") from exc

    intention = int(message.intention)
    if intention not in INTENTION_NAMES:
        raise ProtocolError(f"intention={intention}，协议只允许 1、2、3")

    start_x = int(message.start_pos_x)
    start_y = int(message.start_pos_y)
    if start_x > 0xFFFF or start_y > 0xFFFF:
        raise ProtocolError(
            f"起点 ({start_x}, {start_y}) 超出 0x0307 uint16 坐标范围"
        )

    offset_x = _validate_offsets("offset_x", message.offset_x)
    offset_y = _validate_offsets("offset_y", message.offset_y)

    points: list[PathPoint] = [PathPoint(index=0, x_dm=start_x, y_dm=start_y)]
    x_dm = start_x
    y_dm = start_y
    for index, (delta_x, delta_y) in enumerate(zip(offset_x, offset_y), start=1):
        # 0x0307 explicitly defines every delta relative to the previous point.
        x_dm += delta_x
        y_dm += delta_y
        points.append(PathPoint(index=index, x_dm=x_dm, y_dm=y_dm))

    return DecodedPath(
        intention=intention,
        sender_id=int(message.sender_id),
        points=tuple(points),
        payload_size=len(raw_payload),
    )


def encode_robot_path_plan(
    *,
    intention: int,
    start_x_dm: int,
    start_y_dm: int,
    offset_x: Iterable[int],
    offset_y: Iterable[int],
    sender_id: int,
) -> bytes:
    """Encode a message for tests and offline demo mode."""
    x_values = _validate_offsets("offset_x", offset_x)
    y_values = _validate_offsets("offset_y", offset_y)
    if intention not in INTENTION_NAMES:
        raise ProtocolError(f"intention={intention}，协议只允许 1、2、3")
    if not (0 <= start_x_dm <= 0xFFFF and 0 <= start_y_dm <= 0xFFFF):
        raise ProtocolError("起点必须位于 uint16 范围")

    message = RobotPathPlanInfoMessage()
    message.intention = intention
    message.start_pos_x = start_x_dm
    message.start_pos_y = start_y_dm
    message.offset_x.extend(x_values)
    message.offset_y.extend(y_values)
    message.sender_id = sender_id
    return message.SerializeToString()


def make_demo_payload() -> bytes:
    """Create a visible 50-point path that remains inside a 280 x 150 dm field."""
    offset_x = [5] * DELTA_COUNT
    offset_y: list[int] = []
    for index in range(DELTA_COUNT):
        if index < 10:
            offset_y.append(3)
        elif index < 20:
            offset_y.append(-2)
        elif index < 30:
            offset_y.append(3)
        elif index < 40:
            offset_y.append(-3)
        else:
            offset_y.append(2)

    return encode_robot_path_plan(
        intention=3,
        start_x_dm=15,
        start_y_dm=30,
        offset_x=offset_x,
        offset_y=offset_y,
        sender_id=7,
    )
