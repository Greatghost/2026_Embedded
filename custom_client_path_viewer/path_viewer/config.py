from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class MqttConfig:
    host: str = "192.168.12.1"
    port: int = 3333
    local_bind_ip: str = "192.168.12.2"
    client_id: str = "7"
    topic: str = "RobotPathPlanInfo"
    keepalive_seconds: int = 30


@dataclass(frozen=True, slots=True)
class MapConfig:
    field_width_dm: int = 280
    field_height_dm: int = 150
    background_image: str = "assets/rmuc_2026_official_field_map.png"
    mirror_x: bool = False
    mirror_y: bool = False
    arrow_every: int = 4
    show_all_labels: bool = False


@dataclass(frozen=True, slots=True)
class AppConfig:
    mqtt: MqttConfig
    map: MapConfig
    source_path: Path


DEFAULT_MQTT = MqttConfig()
DEFAULT_MAP = MapConfig()


def _section(data: dict[str, Any], name: str) -> dict[str, Any]:
    value = data.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(f"配置项 {name} 必须是 JSON 对象")
    return value


def load_config(path: str | Path) -> AppConfig:
    source_path = Path(path).expanduser().resolve()
    with source_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("配置文件根节点必须是 JSON 对象")

    mqtt_data = _section(data, "mqtt")
    map_data = _section(data, "map")

    mqtt = MqttConfig(
        host=str(mqtt_data.get("host", DEFAULT_MQTT.host)).strip(),
        port=int(mqtt_data.get("port", DEFAULT_MQTT.port)),
        local_bind_ip=str(
            mqtt_data.get("local_bind_ip", DEFAULT_MQTT.local_bind_ip)
        ).strip(),
        client_id=str(mqtt_data.get("client_id", DEFAULT_MQTT.client_id)).strip(),
        topic=str(mqtt_data.get("topic", DEFAULT_MQTT.topic)).strip(),
        keepalive_seconds=int(
            mqtt_data.get("keepalive_seconds", DEFAULT_MQTT.keepalive_seconds)
        ),
    )
    map_config = MapConfig(
        field_width_dm=int(
            map_data.get("field_width_dm", DEFAULT_MAP.field_width_dm)
        ),
        field_height_dm=int(
            map_data.get("field_height_dm", DEFAULT_MAP.field_height_dm)
        ),
        background_image=str(
            map_data.get("background_image", DEFAULT_MAP.background_image)
        ).strip(),
        mirror_x=bool(map_data.get("mirror_x", DEFAULT_MAP.mirror_x)),
        mirror_y=bool(map_data.get("mirror_y", DEFAULT_MAP.mirror_y)),
        arrow_every=int(map_data.get("arrow_every", DEFAULT_MAP.arrow_every)),
        show_all_labels=bool(
            map_data.get("show_all_labels", DEFAULT_MAP.show_all_labels)
        ),
    )

    if not mqtt.host:
        raise ValueError("mqtt.host 不能为空")
    if not (1 <= mqtt.port <= 65535):
        raise ValueError("mqtt.port 必须位于 1..65535")
    if not mqtt.client_id:
        raise ValueError("mqtt.client_id 不能为空，应填写 7 或 107")
    if not mqtt.topic:
        raise ValueError("mqtt.topic 不能为空")
    if mqtt.keepalive_seconds <= 0:
        raise ValueError("mqtt.keepalive_seconds 必须大于 0")
    if map_config.field_width_dm <= 0 or map_config.field_height_dm <= 0:
        raise ValueError("场地尺寸必须大于 0")
    if not (1 <= map_config.arrow_every <= 49):
        raise ValueError("map.arrow_every 必须位于 1..49")

    background = map_config.background_image
    if background:
        background_path = Path(background).expanduser()
        if not background_path.is_absolute():
            background_path = source_path.parent / background_path
        map_config = MapConfig(
            field_width_dm=map_config.field_width_dm,
            field_height_dm=map_config.field_height_dm,
            background_image=str(background_path.resolve()),
            mirror_x=map_config.mirror_x,
            mirror_y=map_config.mirror_y,
            arrow_every=map_config.arrow_every,
            show_all_labels=map_config.show_all_labels,
        )

    return AppConfig(mqtt=mqtt, map=map_config, source_path=source_path)
