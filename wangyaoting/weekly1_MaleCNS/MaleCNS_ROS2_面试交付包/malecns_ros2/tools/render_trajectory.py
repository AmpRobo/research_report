"""Draw an animation from recorded Gazebo odometry, not a planned path."""

import csv
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main(csv_path: Path, gif_path: Path, png_path: Path) -> None:
    with csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    times = [float(row["time"]) for row in rows]
    first_done = next((float(row["time"]) for row in rows if row["done"] == "1"), times[-1])
    duration = min(first_done + 2.0, times[-1])
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(font_path, 23)
    small = ImageFont.truetype(font_path, 17)
    box = (60, 75, 840, 450)

    def pt(x, y):
        return (round(box[0] + (x + 4) / 8 * (box[2] - box[0])),
                round(box[3] - (y + 2) / 4 * (box[3] - box[1])))

    frames = []
    idx = 0
    for tick in range(math.ceil(duration * 2) + 1):
        sim_t = tick / 2
        while idx + 1 < len(rows) and times[idx + 1] <= sim_t:
            idx += 1
        row = rows[idx]
        x, y, yaw = (float(row[k]) for k in ("x", "y", "yaw_rad"))
        im = Image.new("RGB", (900, 560), "#f4f7fb")
        d = ImageDraw.Draw(im)
        d.text((35, 20), "Gazebo 实测轨迹：MaleCNS 机器人避障", fill="#192435", font=font)
        d.rectangle(box, fill="white", outline="#aab6c8", width=2)
        for gx in range(-4, 5):
            xx = pt(gx, 0)[0]
            d.line((xx, box[1], xx, box[3]), fill="#edf0f5")
        for gy in range(-2, 3):
            yy = pt(0, gy)[1]
            d.line((box[0], yy, box[2], yy), fill="#edf0f5")
        for cx, cy, sx, sy, color in [(-0.1, 0.30, 0.65, 0.80, "#d8433d"),
                                      (1.15, -0.30, 0.50, 0.55, "#e79b2d")]:
            left, top = pt(cx - sx / 2, cy + sy / 2)
            right, bottom = pt(cx + sx / 2, cy - sy / 2)
            d.rectangle((left, top, right, bottom), fill=color)
        gx, gy = pt(2.5, 0)
        d.ellipse((gx - 14, gy - 14, gx + 14, gy + 14), outline="#21a467", width=4)
        d.text((gx + 20, gy - 12), "目标", fill="#14824f", font=small)
        trace = [pt(float(r["x"]), float(r["y"])) for r in rows[:idx + 1:3]]
        trace.append(pt(x, y))
        if len(trace) > 1:
            d.line(trace, fill="#2778d4", width=4, joint="curve")
        rx, ry = pt(x, y)
        d.ellipse((rx - 12, ry - 12, rx + 12, ry + 12), fill="#175eb8", outline="white", width=3)
        d.line((rx, ry, rx + 24 * math.cos(yaw), ry - 24 * math.sin(yaw)), fill="white", width=4)
        d.text((40, 475), f"仿真 {sim_t:04.1f} 秒   位置 ({x:+.2f}, {y:+.2f}) 米   距目标 {float(row['distance_to_goal']):.2f} 米", fill="#26364b", font=small)
        d.text((40, 510), "蓝线来自真实 Gazebo 里程数据；红/橙色方块是仿真障碍物。", fill="#536478", font=small)
        frames.append(im)
    gif_path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=110, loop=0, optimize=True)
    frames[-1].save(png_path)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
