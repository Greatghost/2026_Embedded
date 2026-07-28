from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

from PIL import Image, ImageDraw, ImageFont

from .config import MapConfig
from .protocol import DecodedPath, PathPoint

BACKGROUND = "#08131f"
PANEL = "#0e2131"
GRID = "#294252"
AXIS = "#6f8795"
TEXT = "#e7f1f5"
MUTED = "#8aa1ad"
ATTACK = "#ff5d68"
DEFEND = "#4ea8ff"
MOVE = "#39d98a"
OUT_OF_BOUNDS = "#ffb454"
SELECTED = "#ffe066"


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf",
        "arialbd.ttf" if bold else "arial.ttf",
        str(
            Path("C:/Windows/Fonts")
            / ("segoeuib.ttf" if bold else "segoeui.ttf")
        ),
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


class PathRenderer:
    def __init__(self, config: MapConfig):
        self.config = config
        self._background: Optional[Image.Image] = None
        self.background_error = ""
        if config.background_image:
            try:
                self._background = Image.open(config.background_image).convert("RGB")
            except (OSError, ValueError) as exc:
                self.background_error = f"Map image error: {exc}"

    def _intent_color(self, path: DecodedPath) -> str:
        if path.intention == 1:
            return ATTACK
        if path.intention == 2:
            return DEFEND
        return MOVE

    def _map_rect(self, width: int, height: int) -> tuple[int, int, int, int]:
        left_margin = 72
        right_margin = 28
        top_margin = 68
        bottom_margin = 62
        available_width = max(100, width - left_margin - right_margin)
        available_height = max(100, height - top_margin - bottom_margin)
        field_aspect = self.config.field_width_dm / self.config.field_height_dm

        map_width = available_width
        map_height = int(round(map_width / field_aspect))
        if map_height > available_height:
            map_height = available_height
            map_width = int(round(map_height * field_aspect))

        left = left_margin + (available_width - map_width) // 2
        top = top_margin + (available_height - map_height) // 2
        return left, top, left + map_width, top + map_height

    def _to_pixel(
        self,
        x_dm: float,
        y_dm: float,
        rect: tuple[int, int, int, int],
    ) -> tuple[float, float]:
        left, top, right, bottom = rect
        x_ratio = min(max(x_dm / self.config.field_width_dm, 0.0), 1.0)
        y_ratio = min(max(y_dm / self.config.field_height_dm, 0.0), 1.0)
        if self.config.mirror_x:
            x_ratio = 1.0 - x_ratio
        if self.config.mirror_y:
            y_ratio = 1.0 - y_ratio
        return (
            left + x_ratio * (right - left),
            bottom - y_ratio * (bottom - top),
        )

    def _is_in_bounds(self, point: PathPoint) -> bool:
        return (
            0 <= point.x_dm <= self.config.field_width_dm
            and 0 <= point.y_dm <= self.config.field_height_dm
        )

    @staticmethod
    def _arrow(
        draw: ImageDraw.ImageDraw,
        start: tuple[float, float],
        end: tuple[float, float],
        color: str,
    ) -> None:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 5:
            return

        ux = dx / length
        uy = dy / length
        arrow_start = (
            start[0] + dx * 0.28,
            start[1] + dy * 0.28,
        )
        tip = (
            start[0] + dx * 0.72,
            start[1] + dy * 0.72,
        )
        draw.line([arrow_start, tip], fill=color, width=3)

        head_length = 10
        head_width = 5
        base_x = tip[0] - ux * head_length
        base_y = tip[1] - uy * head_length
        perpendicular_x = -uy
        perpendicular_y = ux
        draw.polygon(
            [
                tip,
                (
                    base_x + perpendicular_x * head_width,
                    base_y + perpendicular_y * head_width,
                ),
                (
                    base_x - perpendicular_x * head_width,
                    base_y - perpendicular_y * head_width,
                ),
            ],
            fill=color,
        )

    def _draw_coordinate_grid(
        self,
        draw: ImageDraw.ImageDraw,
        rect: tuple[int, int, int, int],
        *,
        line_color: str = GRID,
    ) -> None:
        left, top, right, bottom = rect

        grid_step_dm = 20
        label_font = _font(11)
        for x_dm in range(0, self.config.field_width_dm + 1, grid_step_dm):
            x, _ = self._to_pixel(x_dm, 0, rect)
            draw.line([(x, top), (x, bottom)], fill=line_color, width=1)
            label = f"{x_dm / 10:g}m"
            box = draw.textbbox((0, 0), label, font=label_font)
            draw.text(
                (x - (box[2] - box[0]) / 2, bottom + 8),
                label,
                fill=MUTED,
                font=label_font,
            )

        for y_dm in range(0, self.config.field_height_dm + 1, grid_step_dm):
            _, y = self._to_pixel(0, y_dm, rect)
            draw.line([(left, y), (right, y)], fill=line_color, width=1)
            label = f"{y_dm / 10:g}m"
            box = draw.textbbox((0, 0), label, font=label_font)
            draw.text(
                (left - (box[2] - box[0]) - 9, y - (box[3] - box[1]) / 2),
                label,
                fill=MUTED,
                font=label_font,
            )

        draw.text((right - 14, bottom + 28), "+X", fill=TEXT, font=_font(12, True))
        draw.text((left - 34, top - 8), "+Y", fill=TEXT, font=_font(12, True))
        draw.rectangle(rect, outline=AXIS, width=2)

    def render(
        self,
        path: DecodedPath,
        width: int,
        height: int,
        *,
        selected_index: int | None = None,
        show_all_labels: bool | None = None,
    ) -> Image.Image:
        width = max(640, int(width))
        height = max(420, int(height))
        image = Image.new("RGB", (width, height), BACKGROUND)
        draw = ImageDraw.Draw(image)
        rect = self._map_rect(width, height)
        left, top, right, bottom = rect

        if self._background is not None:
            fitted = self._background.resize(
                (right - left, bottom - top),
                resample=Image.Resampling.LANCZOS,
            )
            image.paste(fitted, (left, top))
            overlay = Image.new(
                "RGBA",
                (right - left, bottom - top),
                (3, 15, 24, 48),
            )
            image.paste(overlay, (left, top), overlay)
            draw = ImageDraw.Draw(image)
            self._draw_coordinate_grid(
                draw,
                rect,
                line_color="#607582",
            )
        else:
            draw.rectangle(rect, fill=PANEL)
            self._draw_coordinate_grid(draw, rect)

        color = self._intent_color(path)
        pixels = [
            self._to_pixel(point.x_dm, point.y_dm, rect) for point in path.points
        ]
        draw.line(pixels, fill="#061015", width=8, joint="curve")
        draw.line(pixels, fill=color, width=4, joint="curve")

        arrow_step = self.config.arrow_every
        arrow_end_indices = list(range(arrow_step, len(pixels), arrow_step))
        if not arrow_end_indices or arrow_end_indices[-1] != len(pixels) - 1:
            arrow_end_indices.append(len(pixels) - 1)
        for end_index in arrow_end_indices:
            start_index = max(0, end_index - arrow_step)
            self._arrow(
                draw,
                pixels[start_index],
                pixels[end_index],
                "#ffffff",
            )

        label_all = (
            self.config.show_all_labels
            if show_all_labels is None
            else show_all_labels
        )
        point_font = _font(10, True)
        for point, (pixel_x, pixel_y) in zip(path.points, pixels):
            in_bounds = self._is_in_bounds(point)
            point_color = color if in_bounds else OUT_OF_BOUNDS
            radius = 4
            if point.index in (0, len(path.points) - 1):
                radius = 7
            if point.index == selected_index:
                draw.ellipse(
                    (
                        pixel_x - 11,
                        pixel_y - 11,
                        pixel_x + 11,
                        pixel_y + 11,
                    ),
                    outline=SELECTED,
                    width=3,
                )
                radius = 6
            draw.ellipse(
                (
                    pixel_x - radius,
                    pixel_y - radius,
                    pixel_x + radius,
                    pixel_y + radius,
                ),
                fill=point_color,
                outline="#ffffff",
                width=1,
            )
            if label_all or point.index in (0, len(path.points) - 1) or point.index % 5 == 0:
                draw.text(
                    (pixel_x + 6, pixel_y - 16),
                    f"P{point.index}",
                    fill=TEXT,
                    font=point_font,
                    stroke_width=2,
                    stroke_fill=BACKGROUND,
                )

        title_font = _font(20, True)
        meta_font = _font(13)
        draw.text((22, 16), "RobotPathPlanInfo", fill=TEXT, font=title_font)
        draw.text(
            (250, 21),
            (
                f"Intent: {path.intention}  |  Sender: {path.sender_id}  |  "
                f"Points: {len(path.points)}  |  Payload: {path.payload_size} B"
            ),
            fill=MUTED,
            font=meta_font,
        )

        legend_y = height - 28
        draw.ellipse((22, legend_y, 30, legend_y + 8), fill=color)
        draw.text(
            (38, legend_y - 4),
            "path point",
            fill=MUTED,
            font=_font(11),
        )
        draw.ellipse((132, legend_y, 140, legend_y + 8), fill=OUT_OF_BOUNDS)
        draw.text(
            (148, legend_y - 4),
            "clamped out-of-field point",
            fill=MUTED,
            font=_font(11),
        )
        draw.text(
            (right - 172, legend_y - 4),
            "white arrows = direction",
            fill=MUTED,
            font=_font(11),
        )

        if self.background_error:
            draw.text(
                (22, 45),
                self.background_error,
                fill=OUT_OF_BOUNDS,
                font=_font(11),
            )

        return image
