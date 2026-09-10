# -*- coding: utf-8 -*-
"""按参考图片建立双侧夹持机构的 SolidWorks 概念装配体。"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

SCRIPT_DIR = r"C:\Users\Sui\.codex\skills\solidworks-automation-skill\scripts"
sys.path.insert(0, SCRIPT_DIR)

from sw_connect import connect_solidworks, find_template, mm, save_document
from sw_part import sketch, sketch_circle, sketch_corner_rectangle, sketch_slot, extrude_boss, extrude_cut
from sw_assembly import add_component


OUT_DIR = Path(r"D:\Work_files\Vessel intervention Robot\260316\SW_reproduced")
PART_DIR = OUT_DIR / "parts"


def set_color(model, rgb):
    """设置零件显示颜色。"""
    r, g, b = rgb
    try:
        model.MaterialPropertyValues = [
            float(r), float(g), float(b),
            0.25, 0.55, 0.35,
            0.0, 0.5, 0.0,
        ]
    except Exception:
        pass


def save_part(sw, model, path):
    """重建并保存零件。"""
    model.ForceRebuild3(False)
    model.ViewZoomtofit2()
    if not save_document(model, str(path)):
        raise RuntimeError(f"零件保存失败: {path}")


def make_box(sw, template, name, w, h, d, color):
    """建立以原点为中心的矩形实体。"""
    model = sw.NewDocument(template, 0, 0, 0)
    if model is None:
        raise RuntimeError(f"无法创建零件: {name}")
    with sketch(model, "Front Plane") as sketch_name:
        sketch_corner_rectangle(model, -mm(w / 2), -mm(h / 2), mm(w / 2), mm(h / 2))
    if extrude_boss(model, sketch_name, mm(d)) is None:
        raise RuntimeError(f"矩形拉伸失败: {name}")
    set_color(model, color)
    path = PART_DIR / f"{name}.sldprt"
    save_part(sw, model, path)
    return path


def make_plate_with_slot(sw, template, name, w, h, d, slot_w, slot_h, color):
    """建立带中间长圆槽的板件。"""
    model = sw.NewDocument(template, 0, 0, 0)
    if model is None:
        raise RuntimeError(f"无法创建零件: {name}")
    with sketch(model, "Front Plane") as sketch_name:
        sketch_corner_rectangle(model, -mm(w / 2), -mm(h / 2), mm(w / 2), mm(h / 2))
    if extrude_boss(model, sketch_name, mm(d)) is None:
        raise RuntimeError(f"板件拉伸失败: {name}")
    with sketch(model, "Front Plane") as cut_sketch:
        sketch_slot(
            model,
            -mm(slot_h / 2),
            0,
            mm(slot_h / 2),
            0,
            mm(slot_w / 2),
        )
    if extrude_cut(model, cut_sketch, 0) is None:
        raise RuntimeError(f"槽口切除失败: {name}")
    set_color(model, color)
    path = PART_DIR / f"{name}.sldprt"
    save_part(sw, model, path)
    return path


def make_cylinder_z(sw, template, name, diameter, length, color):
    """建立轴线沿 Z 方向的圆柱件。"""
    model = sw.NewDocument(template, 0, 0, 0)
    if model is None:
        raise RuntimeError(f"无法创建圆柱零件: {name}")
    with sketch(model, "Front Plane") as sketch_name:
        sketch_circle(model, 0, 0, mm(diameter / 2))
    if extrude_boss(model, sketch_name, mm(length)) is None:
        raise RuntimeError(f"圆柱拉伸失败: {name}")
    set_color(model, color)
    path = PART_DIR / f"{name}.sldprt"
    save_part(sw, model, path)
    return path


def make_cylinder_x(sw, template, name, diameter, length, color):
    """建立轴线沿 X 方向的圆柱件，用作弹簧占位体。"""
    model = sw.NewDocument(template, 0, 0, 0)
    if model is None:
        raise RuntimeError(f"无法创建 X 轴圆柱零件: {name}")
    with sketch(model, "Right Plane") as sketch_name:
        sketch_circle(model, 0, 0, mm(diameter / 2))
    if extrude_boss(model, sketch_name, mm(length)) is None:
        raise RuntimeError(f"X 轴圆柱拉伸失败: {name}")
    set_color(model, color)
    path = PART_DIR / f"{name}.sldprt"
    save_part(sw, model, path)
    return path


def close_other_docs(sw, keep_title):
    """关闭已保存的零件文档，避免装配插入时文档过多。"""
    try:
        docs = sw.GetDocuments()
    except Exception:
        return
    if not docs:
        return
    for doc in list(docs):
        try:
            title = doc.GetTitle()
            if title != keep_title:
                sw.CloseDoc(title)
        except Exception:
            continue


def build():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    PART_DIR.mkdir(parents=True, exist_ok=True)

    sw, active = connect_solidworks(wait_seconds=2, visible=True)
    part_template = find_template(sw, "part")
    asm_template = find_template(sw, "assembly")

    colors = {
        "dark": (0.16, 0.18, 0.20),
        "teal": (0.05, 0.48, 0.52),
        "brown": (0.45, 0.18, 0.12),
        "red": (0.82, 0.03, 0.03),
        "steel": (0.48, 0.50, 0.53),
        "blue": (0.03, 0.16, 0.72),
    }

    parts = {}
    parts["housing"] = make_box(sw, part_template, "housing", 86, 74, 22, colors["dark"])
    parts["inner_block"] = make_box(sw, part_template, "inner_block", 38, 78, 28, colors["teal"])
    parts["crossbar"] = make_box(sw, part_template, "crossbar", 116, 14, 18, colors["steel"])
    parts["jaw_plate"] = make_plate_with_slot(
        sw, part_template, "jaw_plate", 48, 58, 16, 18, 28, colors["steel"]
    )
    parts["link"] = make_box(sw, part_template, "link", 14, 52, 10, colors["brown"])
    parts["upper_rail"] = make_box(sw, part_template, "upper_rail", 128, 12, 16, colors["brown"])
    parts["upright"] = make_box(sw, part_template, "upright", 32, 128, 20, colors["brown"])
    parts["spring"] = make_cylinder_x(sw, part_template, "spring_placeholder", 13, 42, colors["red"])
    parts["pin"] = make_cylinder_z(sw, part_template, "pin", 9, 32, colors["blue"])
    parts["center_hub"] = make_cylinder_z(sw, part_template, "center_hub", 24, 24, colors["brown"])
    parts["center_plate"] = make_box(sw, part_template, "center_plate", 34, 52, 18, colors["steel"])
    parts["top_block"] = make_box(sw, part_template, "top_block", 58, 30, 20, colors["brown"])

    assembly = sw.NewDocument(asm_template, 0, 0, 0)
    if assembly is None:
        raise RuntimeError("无法创建装配体")
    assembly_path = OUT_DIR / "dual_clamp_concept.sldasm"

    # 第一版按图片比例放置，坐标单位为 mm，Z 方向用来错开前后层级。
    placements = [
        ("housing", -108, 0, 0, True),
        ("housing", 108, 0, 0, False),
        ("inner_block", -60, 0, 10, False),
        ("inner_block", 60, 0, 10, False),
        ("crossbar", -104, -5, -16, False),
        ("crossbar", 104, -5, -16, False),
        ("jaw_plate", -72, 0, 26, False),
        ("jaw_plate", 72, 0, 26, False),
        ("center_plate", -22, 0, 34, False),
        ("center_plate", 22, 0, 34, False),
        ("center_hub", 0, 0, 48, False),
        ("upper_rail", 0, 78, 0, False),
        ("upright", 0, 138, 0, False),
        ("top_block", -28, 94, 18, False),
        ("top_block", 28, 94, 18, False),
        ("link", -38, 91, 34, False),
        ("link", 38, 91, 34, False),
        ("spring", -150, 42, 22, False),
        ("spring", 108, 42, 22, False),
        ("spring", -150, -42, 22, False),
        ("spring", 108, -42, 22, False),
        ("pin", -72, 0, 42, False),
        ("pin", 72, 0, 42, False),
        ("pin", -38, 91, 42, False),
        ("pin", 38, 91, 42, False),
    ]

    components = []
    for key, x, y, z, fixed in placements:
        component = add_component(
            assembly,
            str(parts[key]),
            x=mm(x),
            y=mm(y),
            z=mm(z),
            sw=sw,
        )
        if component is None:
            raise RuntimeError(f"组件插入失败: {key} @ ({x}, {y}, {z})")
        components.append(component)

    assembly.ForceRebuild3(False)
    assembly.ViewZoomtofit2()
    if not save_document(assembly, str(assembly_path)):
        raise RuntimeError("装配体保存失败")

    # 保存一个可交换格式，便于快速检查整体几何。
    try:
        errors = 0
        warnings = 0
        assembly.Extension.SaveAs(
            str(OUT_DIR / "dual_clamp_concept.step"),
            0,
            1,
            None,
            errors,
            warnings,
        )
    except Exception:
        pass

    print(f"输出目录: {OUT_DIR}")
    print(f"装配体: {assembly_path}")
    print(f"零件数量: {len(parts)}")
    print(f"组件实例数量: {len(components)}")
    return assembly_path


if __name__ == "__main__":
    build()
