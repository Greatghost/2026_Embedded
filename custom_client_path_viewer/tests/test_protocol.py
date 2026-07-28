from __future__ import annotations

import unittest

from path_viewer.protocol import (
    DELTA_COUNT,
    POINT_COUNT,
    ProtocolError,
    decode_robot_path_plan,
    encode_robot_path_plan,
    make_demo_payload,
)


class ProtocolTests(unittest.TestCase):
    def test_demo_payload_decodes_to_exactly_50_points(self) -> None:
        decoded = decode_robot_path_plan(make_demo_payload())

        self.assertEqual(decoded.intention, 3)
        self.assertEqual(decoded.sender_id, 7)
        self.assertEqual(len(decoded.points), POINT_COUNT)
        self.assertEqual((decoded.points[0].x_dm, decoded.points[0].y_dm), (15, 30))
        self.assertEqual(
            (decoded.points[-1].x_dm, decoded.points[-1].y_dm),
            (260, 58),
        )

    def test_offsets_are_cumulative_from_previous_point(self) -> None:
        offset_x = [0] * DELTA_COUNT
        offset_y = [0] * DELTA_COUNT
        offset_x[0:3] = [5, 0, -2]
        offset_y[0:3] = [0, 6, 0]
        payload = encode_robot_path_plan(
            intention=1,
            start_x_dm=50,
            start_y_dm=30,
            offset_x=offset_x,
            offset_y=offset_y,
            sender_id=107,
        )

        decoded = decode_robot_path_plan(payload)
        coordinates = [(point.x_dm, point.y_dm) for point in decoded.points[:4]]
        self.assertEqual(
            coordinates,
            [(50, 30), (55, 30), (55, 36), (53, 36)],
        )

    def test_negative_int32_offsets_round_trip(self) -> None:
        offset_x = [-128] + [0] * (DELTA_COUNT - 1)
        offset_y = [127] + [0] * (DELTA_COUNT - 1)
        payload = encode_robot_path_plan(
            intention=2,
            start_x_dm=200,
            start_y_dm=10,
            offset_x=offset_x,
            offset_y=offset_y,
            sender_id=7,
        )

        decoded = decode_robot_path_plan(payload)
        self.assertEqual((decoded.points[1].x_dm, decoded.points[1].y_dm), (72, 137))

    def test_invalid_delta_length_is_rejected(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "固定为 49"):
            encode_robot_path_plan(
                intention=3,
                start_x_dm=0,
                start_y_dm=0,
                offset_x=[0] * 48,
                offset_y=[0] * 49,
                sender_id=7,
            )

    def test_invalid_delta_range_is_rejected(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "超出 int8"):
            encode_robot_path_plan(
                intention=3,
                start_x_dm=0,
                start_y_dm=0,
                offset_x=[128] + [0] * 48,
                offset_y=[0] * 49,
                sender_id=7,
            )


if __name__ == "__main__":
    unittest.main()
