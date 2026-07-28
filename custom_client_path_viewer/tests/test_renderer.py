from __future__ import annotations

import unittest
from pathlib import Path

from PIL import Image

from path_viewer.config import load_config
from path_viewer.protocol import decode_robot_path_plan, make_demo_payload
from path_viewer.renderer import PathRenderer


class RendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        project_dir = Path(__file__).resolve().parents[1]
        cls.config = load_config(project_dir / "config.example.json")

    def test_demo_render_has_requested_size_and_path_colors(self) -> None:
        path = decode_robot_path_plan(make_demo_payload())
        image = PathRenderer(self.config.map).render(path, 1000, 600)

        self.assertEqual(image.size, (1000, 600))
        colors = image.getcolors(maxcolors=image.width * image.height)
        self.assertIsNotNone(colors)
        color_values = {rgb for _, rgb in colors}
        self.assertIn((57, 217, 138), color_values)  # MOVE
        self.assertIn((255, 255, 255), color_values)  # direction arrows

    def test_official_map_is_bundled_and_calibrated_to_28_by_15(self) -> None:
        background = Path(self.config.map.background_image)
        self.assertTrue(background.is_file())
        with Image.open(background) as image:
            self.assertEqual(image.size, (1400, 750))

        renderer = PathRenderer(self.config.map)
        self.assertEqual(renderer.background_error, "")
        rect = renderer._map_rect(1000, 600)
        left, top, right, bottom = rect
        self.assertEqual(renderer._to_pixel(0, 0, rect), (left, bottom))
        self.assertEqual(
            renderer._to_pixel(280, 150, rect),
            (right, top),
        )


if __name__ == "__main__":
    unittest.main()
