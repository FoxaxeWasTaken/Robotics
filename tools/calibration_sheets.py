#!/usr/bin/env python3
"""calibration_sheets.py - the printed calibration sheets, A4 PDF in vector form so 100 % scale is exact, in
the control image (it needs cv2.aruco):

    tools/so101 run --out /tmp/sheets 'python3 /repo/tools/calibration_sheets.py /out'

aruco_markers.pdf: the table's markers (ids 0-7, 75 mm, a cut line at 100 mm, 05 §5.2). charuco_board.pdf:
the board of so101_perception.intrinsics (05 §3.2). connect4_markers.pdf: the game's markers, the grid's four
and the dispenser's four, as connect4_grid.yaml says them (phase2/P2-02 §1, hardware/connect4).
"""

import sys
from pathlib import Path

import cv2
import yaml

sys.path.insert(0, "/repo/ros2_ws/src/so101_perception")
GAME = Path(__file__).resolve().parents[1] / "ros2_ws/src/so101_connect4/config/connect4_grid.yaml"
# Where each marker goes, in the order of its ids in the YAML (layout.marker_centers,
# layout.dispenser_marker_centers)
ON_THE_GRID = ["post on the arm's side, top", "far post, top",
               "post on the arm's side, bottom", "far post, bottom"]  # fmt: skip
ON_THE_DISPENSER = ["beside the rails, station end", "far from the rails, station end",
                    "beside the rails, arm end", "far from the rails, arm end"]  # fmt: skip

MM = 72 / 25.4
A4 = 210.0, 297.0
SIDE, PLANE, CELLS = 75.0, 100.0, 6


def text(x, y, size, s):
    s = s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    return f"BT /F1 {size} Tf {x * MM:.3f} {y * MM:.3f} Td ({s}) Tj ET\n"


def rectangles(image, x0, top, pixel):
    """The dark runs of a gray image as filled rectangles: top-left corner at (x0, top) mm, pixel mm wide."""
    ops = "0 g\n"
    for r, row in enumerate(image < 128):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c < len(row) and row[c]:
                    c += 1
                x, y = (x0 + start * pixel) * MM, (top - (r + 1) * pixel) * MM
                ops += f"{x:.4f} {y:.4f} {(c - start) * pixel * MM:.4f} {pixel * MM:.4f} re\n"
            c += 1
    return ops + "f\n"


def marker(aruco, i, cx, cy):
    cell = SIDE / CELLS
    ops = rectangles(cv2.aruco.generateImageMarker(aruco, i, CELLS), cx - SIDE / 2, cy + SIDE / 2, cell)
    x1, y1 = cx - PLANE / 2, cy - PLANE / 2
    ops += f"{0.25 * MM:.3f} w {x1 * MM:.4f} {y1 * MM:.4f} {PLANE * MM:.4f} {PLANE * MM:.4f} re S\n"
    ly, ax = cy + PLANE / 2 + 5, cx - PLANE / 2 + 3  # above the line: cut away with the sheet
    ops += f"{ax * MM:.3f} {(ly - 1) * MM:.3f} m {(ax + 3) * MM:.3f} {(ly + 7) * MM:.3f} l "
    ops += f"{(ax + 6) * MM:.3f} {(ly - 1) * MM:.3f} l f\n"
    ops += text(cx - PLANE / 2 + 12, ly + 3.5, 13, f"ID {i}")
    return ops + text(cx - PLANE / 2 + 12, ly - 0.5, 8, "Top of the marker towards +x (the front of the arm)")


def write_pdf(path, pages, size):
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        None,
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>",
    ]
    kids = []
    for ops in pages:
        data = ops.encode("cp1252")
        objects.append(b"<< /Length %d >>\nstream\n%s\nendstream" % (len(data), data))
        objects.append(
            b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Resources << /Font << /F1 3 0 R >> >> "
            b"/Contents %d 0 R >>" % (size[0] * MM, size[1] * MM, len(objects))
        )
        kids.append(b"%d 0 R" % len(objects))
    objects[1] = b"<< /Type /Pages /Kids [%s] /Count %d >>" % (b" ".join(kids), len(kids))
    pdf, offsets = b"%PDF-1.4\n", []
    for n, o in enumerate(objects, 1):
        offsets.append(len(pdf))
        pdf += b"%d 0 obj\n%s\nendobj\n" % (n, o)
    xref = len(pdf)
    pdf += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objects) + 1)
    pdf += b"".join(b"%010d 00000 n \n" % o for o in offsets)
    path.write_bytes(
        pdf + b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objects) + 1, xref)
    )


def markers_pdf(path):
    aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pages = []
    for p in range(4):
        ops = marker(aruco, 2 * p, A4[0] / 2, 222.0) + marker(aruco, 2 * p + 1, A4[0] / 2, 82.0)
        ops += text(15, 12, 8, "Print at 100 %, never fit to page. Black square = 75 mm, line = 100 mm.")
        ops += text(15, 8, 8, "Before cutting: copy the ID and the arrow on the back.")
        pages.append(ops)
    write_pdf(path, pages, A4)


def board_pdf(path):
    from so101_perception.intrinsics import SQUARE_M, SQUARES, board

    pixel = 0.5  # mm: 76 px per 38 mm square, 56 px per 28 mm marker, 8 px per bit, all whole
    square = round(SQUARE_M * 1000 / pixel)
    image = board().generateImage((SQUARES[0] * square, SQUARES[1] * square), marginSize=0, borderBits=1)
    width, height = A4[1], A4[0]  # landscape
    x0 = (width - SQUARES[0] * square * pixel) / 2
    top = height - (height - SQUARES[1] * square * pixel) / 2 + 3
    ops = rectangles(image, x0, top, pixel)
    note = "Print at 100 %, never fit to page. A black square = 38 mm: measure it. Glue it on cardboard."
    ops += text(x0, 6, 9, note)
    write_pdf(path, [ops], (width, height))


def game_marker(aruco, i, cx, cy, side, plane, where):
    """A marker of the game: its black square, the cut line of its white one, and above the line an arrow
    for its top, its id and its place."""
    image = cv2.aruco.generateImageMarker(aruco, i, CELLS)
    ops = rectangles(image, cx - side / 2, cy + side / 2, side / CELLS)
    x1, y1 = cx - plane / 2, cy - plane / 2
    ops += f"{0.25 * MM:.3f} w {x1 * MM:.4f} {y1 * MM:.4f} {plane * MM:.4f} {plane * MM:.4f} re S\n"
    ly = cy + plane / 2 + 3
    ops += f"{x1 * MM:.3f} {ly * MM:.3f} m {(x1 + 2) * MM:.3f} {(ly + 5) * MM:.3f} l "
    ops += f"{(x1 + 4) * MM:.3f} {ly * MM:.3f} l f\n"
    return ops + text(x1 + 6, ly + 1, 8, f"ID {i}: {where}")


def game_pdf(path):
    layout = yaml.safe_load(GAME.read_text())
    grid, dispenser = layout["markers"], layout["dispenser"]["markers"]
    aruco = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, grid["dictionary"]))
    ops = text(25, 278, 11, "Grid: front face of the posts")
    ops += text(119, 278, 11, "Dispenser: marker plate")
    for cx, markers, places in ((58.0, grid, ON_THE_GRID), (152.0, dispenser, ON_THE_DISPENSER)):
        side, plane = markers["size"] * 1000, markers["plate"] * 1000
        for row, (i, where) in enumerate(zip(markers["ids"], places, strict=True)):
            ops += game_marker(aruco, i, cx, 235.0 - 58.0 * row, side, plane, where)
    side, plane = grid["size"] * 1000, grid["plate"] * 1000
    notes = [
        f"Print at 100 %, never fit to page. Black square = {side:g} mm, cut line = {plane:g} mm.",
        "The arrow shows the marker's top. Grid: top up. Dispenser: top towards the rails.",
        "Before cutting: copy the ID and the arrow on the back. Glue it flat on the engraved outline.",
    ]
    for line, note in enumerate(notes):
        ops += text(15, 20 - 4 * line, 8, note)
    write_pdf(path, [ops], A4)


if __name__ == "__main__":
    out = Path(sys.argv[1])
    markers_pdf(out / "aruco_markers.pdf")
    board_pdf(out / "charuco_board.pdf")
    game_pdf(out / "connect4_markers.pdf")
