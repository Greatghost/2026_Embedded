from __future__ import annotations

import threading
from collections.abc import Callable

import paho.mqtt.client as mqtt

from .config import MqttConfig
from .protocol import DecodedPath, ProtocolError, decode_robot_path_plan

PathCallback = Callable[[DecodedPath], None]
StatusCallback = Callable[[str, str], None]


class MqttPathSubscriber:
    def __init__(
        self,
        config: MqttConfig,
        on_path: PathCallback,
        on_status: StatusCallback,
    ):
        self.config = config
        self._on_path = on_path
        self._on_status = on_status
        self._connect_lock = threading.Lock()
        self._connect_in_progress = False
        self._connected = False
        self._loop_started = False

        self._client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=config.client_id,
            clean_session=True,
            protocol=mqtt.MQTTv311,
            transport="tcp",
        )
        self._client.on_connect = self._handle_connect
        self._client.on_disconnect = self._handle_disconnect
        self._client.on_message = self._handle_message
        self._client.on_subscribe = self._handle_subscribe
        self._client.reconnect_delay_set(min_delay=1, max_delay=5)

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        with self._connect_lock:
            if self._connected or self._connect_in_progress:
                return
            self._connect_in_progress = True
        self._on_status(
            "connecting",
            (
                f"正在连接 mqtt://{self.config.host}:{self.config.port}，"
                f"clientID={self.config.client_id}"
            ),
        )
        threading.Thread(
            target=self._connect_worker,
            name="mqtt-connect",
            daemon=True,
        ).start()

    def _connect_worker(self) -> None:
        try:
            self._client.connect(
                host=self.config.host,
                port=self.config.port,
                keepalive=self.config.keepalive_seconds,
                bind_address=self.config.local_bind_ip,
            )
            self._client.loop_start()
            self._loop_started = True
        except OSError as exc:
            self._on_status(
                "error",
                (
                    f"MQTT 连接失败: {exc}。请检查本机是否配置 "
                    f"{self.config.local_bind_ip} 以及裁判服务器链路。"
                ),
            )
        except Exception as exc:  # paho may raise ValueError for invalid settings
            self._on_status("error", f"MQTT 连接失败: {exc}")
        finally:
            with self._connect_lock:
                self._connect_in_progress = False

    def disconnect(self) -> None:
        try:
            if self._connected or self._loop_started:
                self._client.disconnect()
        finally:
            if self._loop_started:
                self._client.loop_stop()
                self._loop_started = False
            self._connected = False
            self._on_status("disconnected", "已断开 MQTT")

    def _handle_connect(
        self,
        client: mqtt.Client,
        userdata,
        flags: mqtt.ConnectFlags,
        reason_code: mqtt.ReasonCode,
        properties,
    ) -> None:
        del userdata, flags, properties
        if reason_code.is_failure:
            self._connected = False
            self._on_status("error", f"MQTT 拒绝连接: {reason_code}")
            return

        self._connected = True
        self._on_status("connected", "MQTT 已连接，正在订阅轨迹主题")
        client.subscribe(self.config.topic, qos=1)

    def _handle_subscribe(
        self,
        client: mqtt.Client,
        userdata,
        mid: int,
        reason_code_list: list[mqtt.ReasonCode],
        properties,
    ) -> None:
        del client, userdata, mid, properties
        if any(code.is_failure for code in reason_code_list):
            self._on_status(
                "error",
                f"订阅 {self.config.topic} 失败: {reason_code_list}",
            )
            return
        self._on_status(
            "subscribed",
            f"已订阅 {self.config.topic}（QoS 1），等待服务器触发消息",
        )

    def _handle_disconnect(
        self,
        client: mqtt.Client,
        userdata,
        disconnect_flags: mqtt.DisconnectFlags,
        reason_code: mqtt.ReasonCode,
        properties,
    ) -> None:
        del client, userdata, disconnect_flags, properties
        self._connected = False
        if reason_code.is_failure:
            self._on_status("error", f"MQTT 异常断开: {reason_code}")
        else:
            self._on_status("disconnected", "MQTT 连接已关闭")

    def _handle_message(
        self,
        client: mqtt.Client,
        userdata,
        message: mqtt.MQTTMessage,
    ) -> None:
        del client, userdata
        if message.topic != self.config.topic:
            return
        try:
            decoded = decode_robot_path_plan(message.payload)
        except ProtocolError as exc:
            self._on_status(
                "error",
                f"{message.topic} 数据无效: {exc}",
            )
            return
        self._on_path(decoded)
        self._on_status(
            "message",
            (
                f"收到 {len(decoded.points)} 点轨迹："
                f"{decoded.intention_name}，sender={decoded.sender_id}，"
                f"payload={decoded.payload_size} B"
            ),
        )
