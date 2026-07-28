from __future__ import annotations

import tkinter as tk
import unittest
from pathlib import Path

from path_viewer.app import PathViewerApp
from path_viewer.config import load_config


class AppSmokeTests(unittest.TestCase):
    def test_demo_populates_map_and_50_row_table(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"Tk display unavailable: {exc}")
            return

        root.withdraw()
        project_dir = Path(__file__).resolve().parents[1]
        app = PathViewerApp(
            root,
            load_config(project_dir / "config.example.json"),
            demo=False,
        )
        try:
            root.update_idletasks()
            app.show_demo()
            app._render()
            self.assertIsNotNone(app.current_path)
            self.assertEqual(len(app.current_path.points), 50)
            self.assertEqual(len(app.point_tree.get_children()), 50)
            self.assertIsNotNone(app._tk_image)
        finally:
            app.subscriber.disconnect()
            root.destroy()


if __name__ == "__main__":
    unittest.main()
