from __future__ import annotations

import argparse
import sys
import tkinter as tk
from pathlib import Path

from path_viewer.app import PathViewerApp
from path_viewer.config import load_config
from path_viewer.protocol import decode_robot_path_plan, make_demo_payload
from path_viewer.renderer import PathRenderer

PROJECT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RoboMaster RobotPathPlanInfo MQTT map viewer"
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_DIR / "config.json"),
        help="JSON 配置文件路径",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="启动后立即显示离线50点演示轨迹",
    )
    parser.add_argument(
        "--export-demo",
        metavar="PNG",
        help="无界面导出演示轨迹 PNG 后退出",
    )
    return parser.parse_args()


def resolve_config(requested: str) -> Path:
    path = Path(requested).expanduser()
    if path.exists():
        return path
    default_path = PROJECT_DIR / "config.json"
    if path.resolve() == default_path.resolve():
        example = PROJECT_DIR / "config.example.json"
        print(
            "提示：未找到 config.json，当前使用 config.example.json。"
            "实机连接前请复制并确认 client_id。",
            file=sys.stderr,
        )
        return example
    raise FileNotFoundError(f"配置文件不存在: {path}")


def main() -> int:
    args = parse_args()
    try:
        config = load_config(resolve_config(args.config))
    except (OSError, ValueError) as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2

    if args.export_demo:
        path = decode_robot_path_plan(make_demo_payload())
        renderer = PathRenderer(config.map)
        image = renderer.render(path, width=1280, height=760)
        output = Path(args.export_demo).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output)
        print(output)
        return 0

    root = tk.Tk()
    PathViewerApp(root, config, demo=args.demo)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
