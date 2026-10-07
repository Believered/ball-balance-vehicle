from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "artifacts" / "report_revision" / "media"
FONT_REGULAR = Path(r"C:\Windows\Fonts\msyh.ttc")
FONT_BOLD = Path(r"C:\Windows\Fonts\msyhbd.ttc")


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold and FONT_BOLD.exists() else FONT_REGULAR
    return ImageFont.truetype(str(path), size)


def centered_text(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int],
                  text: str, size: int = 34, bold: bool = False,
                  fill: str = "#1f2937") -> None:
    fnt = font(size, bold)
    lines = text.split("\n")
    spacing = 8
    line_boxes = [draw.textbbox((0, 0), line, font=fnt) for line in lines]
    heights = [b[3] - b[1] for b in line_boxes]
    total_h = sum(heights) + spacing * (len(lines) - 1)
    y = box[1] + (box[3] - box[1] - total_h) / 2
    for line, bounds, height in zip(lines, line_boxes, heights):
        width = bounds[2] - bounds[0]
        x = box[0] + (box[2] - box[0] - width) / 2
        draw.text((x, y), line, font=fnt, fill=fill)
        y += height + spacing


def rounded_box(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int],
                text: str, fill: str, outline: str = "#334155",
                size: int = 32, bold: bool = False, radius: int = 18) -> None:
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=3)
    centered_text(draw, box, text, size=size, bold=bold)


def arrow(draw: ImageDraw.ImageDraw, start: tuple[int, int], end: tuple[int, int],
          fill: str = "#475569", width: int = 5) -> None:
    draw.line([start, end], fill=fill, width=width)
    x1, y1 = start
    x2, y2 = end
    if abs(x2 - x1) >= abs(y2 - y1):
        direction = 1 if x2 > x1 else -1
        points = [(x2, y2), (x2 - 18 * direction, y2 - 11),
                  (x2 - 18 * direction, y2 + 11)]
    else:
        direction = 1 if y2 > y1 else -1
        points = [(x2, y2), (x2 - 11, y2 - 18 * direction),
                  (x2 + 11, y2 - 18 * direction)]
    draw.polygon(points, fill=fill)


def architecture() -> None:
    image = Image.new("RGB", (1900, 980), "white")
    draw = ImageDraw.Draw(image)
    centered_text(draw, (0, 15, 1900, 90), "车载平衡滚球系统分层控制结构",
                  size=48, bold=True, fill="#0f172a")

    sensor_boxes = [
        ((80, 180, 420, 310), "八路红外灰度\n轨迹横向位置"),
        ((80, 385, 420, 515), "双轮正交编码器\n轮速与里程"),
        ((80, 590, 420, 720), "IMU601\n航向与角速度"),
        ((80, 795, 420, 925), "K230D视觉单元\n球位检测与图传"),
    ]
    colors = ["#e0f2fe", "#dcfce7", "#fef3c7", "#f3e8ff"]
    for (box, label), color in zip(sensor_boxes, colors):
        rounded_box(draw, box, label, color, size=31)

    rounded_box(draw, (650, 140, 1240, 920), "", "#f8fafc", "#0f766e")
    centered_text(draw, (650, 155, 1240, 235), "MSPM0G3507 实时控制器",
                  size=42, bold=True, fill="#0f766e")
    rounded_box(draw, (735, 280, 1155, 395), "任务与路段状态估计",
                "#ccfbf1", size=32, bold=True)
    rounded_box(draw, (735, 455, 1155, 570), "轨迹位置环与航向融合",
                "#dbeafe", size=32, bold=True)
    rounded_box(draw, (735, 630, 1155, 745), "双轮独立速度 PI",
                "#dcfce7", size=32, bold=True)
    rounded_box(draw, (735, 805, 1155, 885), "安全监控与 0.01 s 计时",
                "#fee2e2", size=29, bold=True)

    output_boxes = [
        ((1460, 235, 1810, 365), "TB6612 双路驱动\n差速小车"),
        ((1460, 495, 1810, 625), "摆杆角度执行机构\n钢球位置闭环"),
        ((1460, 755, 1810, 885), "OLED\n模式、状态与时间"),
    ]
    for box, label in output_boxes:
        rounded_box(draw, box, label, "#f1f5f9", size=31)

    for box, _ in sensor_boxes:
        arrow(draw, (box[2], (box[1] + box[3]) // 2),
              (650, (box[1] + box[3]) // 2))
    arrow(draw, (1240, 330), (1460, 300))
    arrow(draw, (1240, 540), (1460, 560))
    arrow(draw, (1240, 830), (1460, 820))

    image.save(OUT / "system_architecture.png", dpi=(220, 220))


def state_machine() -> None:
    image = Image.new("RGB", (1900, 1030), "white")
    draw = ImageDraw.Draw(image)
    centered_text(draw, (0, 15, 1900, 90), "平衡运输速度规划与过点状态机",
                  size=48, bold=True, fill="#0f172a")

    boxes = [
        ((70, 170, 350, 300), "待机\n传感器对齐", "#f1f5f9"),
        ((430, 170, 750, 300), "同步起步\n快轮等待慢轮", "#e0f2fe"),
        ((830, 170, 1130, 300), "S 形加速\n斜率连续", "#dcfce7"),
        ((1210, 170, 1530, 300), "路段跟踪\n直线/弯道调度", "#fef3c7"),
        ((1590, 170, 1840, 300), "里程过点\n冻结计时", "#fce7f3"),
    ]
    for box, label, color in boxes:
        rounded_box(draw, box, label, color, size=31, bold=True)
    for left, right in zip(boxes, boxes[1:]):
        arrow(draw, (left[0][2], 235), (right[0][0], 235))

    rounded_box(draw, (190, 500, 720, 655),
                "AB 任务：通过 B 后进入 BC 弯道\n利用转弯过程缓慢卸载纵向速度",
                "#e0f2fe", size=31, bold=True)
    rounded_box(draw, (1140, 500, 1710, 655),
                "整圈任务：通过 A 后保持匀速\n沿 AB 延伸一段里程再减速",
                "#f3e8ff", size=31, bold=True)
    arrow(draw, (1715, 300), (455, 500))
    arrow(draw, (1715, 300), (1425, 500))

    rounded_box(draw, (570, 790, 1030, 925), "S 形减速\n控制加速度与冲击",
                "#fee2e2", size=32, bold=True)
    rounded_box(draw, (1190, 790, 1600, 925), "速度归零\n高阻释放停车",
                "#f1f5f9", size=32, bold=True)
    arrow(draw, (455, 655), (750, 790))
    arrow(draw, (1425, 655), (850, 790))
    arrow(draw, (1030, 858), (1190, 858))

    centered_text(draw, (110, 955, 1790, 1015),
                  "任务计时与物理停车解耦：过点即锁存成绩，减速在过点后完成。",
                  size=31, bold=True, fill="#9f1239")
    image.save(OUT / "route_state_machine.png", dpi=(220, 220))


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    architecture()
    state_machine()


if __name__ == "__main__":
    main()
