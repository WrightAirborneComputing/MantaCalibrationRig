"""Geometry for the creep curve plot, and an SVG writer for it.

Kept apart from both the GUI and the analysis on purpose. The window draws on a
Tk canvas and the export writes a file, and the two must not drift into showing
different pictures - so the layout is computed once, here, in pixel space, and
both renderers consume the same primitives.

No tkinter, and no plotting library: numpy and matplotlib are absent from
requirements.txt and the README commits to a stock Windows install, so a plot
cannot be the thing that adds a build dependency. SVG needs nothing, scales, and
opens anywhere.
"""

from range_test import band_profile, evaluate_polynomial, fit_polynomial, rms

MARGIN_LEFT = 74
MARGIN_RIGHT = 18
MARGIN_TOP = 18
MARGIN_BOTTOM = 54

# Roles a renderer maps to its own colours, so neither renderer decides what a
# curve means.
ROLE_UP = "up"
ROLE_DOWN = "down"
ROLE_FIT_UP = "fit_up"
ROLE_FIT_DOWN = "fit_down"
ROLE_TRAM = "tram"
ROLE_AXIS = "axis"

MODE_CURVE = "curve"
MODE_DEVIATION = "deviation"

DIRECTIONS = ((1.0, ROLE_UP, ROLE_FIT_UP), (-1.0, ROLE_DOWN, ROLE_FIT_DOWN))

SVG_COLOURS = {
    ROLE_UP: "#185FA5",
    ROLE_DOWN: "#993C1D",
    ROLE_FIT_UP: "#185FA5",
    ROLE_FIT_DOWN: "#993C1D",
    ROLE_TRAM: "#888780",
    ROLE_AXIS: "#5F5E5A",
}


def nice_ticks(low, high, count=6):
    """Round tick values spanning [low, high], or [] if the range is degenerate."""
    if high <= low:
        return []

    raw = (high - low) / float(max(1, count))
    magnitude = 10.0 ** int(_floor_log10(raw))

    for multiple in (1.0, 2.0, 2.5, 5.0, 10.0):
        step = magnitude * multiple
        if step >= raw:
            break

    first = step * int(low / step)
    if first < low:
        first += step

    ticks = []
    value = first
    while value <= high + (step / 1000.0):
        ticks.append(round(value, 6))
        value += step
    return ticks
# def


def _floor_log10(value):
    exponent = 0
    value = abs(value)
    if value <= 0:
        return 0
    while value < 1.0:
        value *= 10.0
        exponent -= 1
    while value >= 10.0:
        value /= 10.0
        exponent += 1
    return exponent
# def


class PlotBox:
    """Maps data coordinates to pixels, with y increasing upward as drawn."""

    def __init__(self, x_range, y_range, width, height):
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        self.width = width
        self.height = height

        self.left = MARGIN_LEFT
        self.right = max(MARGIN_LEFT + 1, width - MARGIN_RIGHT)
        self.top = MARGIN_TOP
        self.bottom = max(MARGIN_TOP + 1, height - MARGIN_BOTTOM)

    def x(self, value):
        span = self.x_max - self.x_min
        if span <= 0:
            return self.left
        fraction = (float(value) - self.x_min) / span
        return self.left + fraction * (self.right - self.left)
    # def

    def y(self, value):
        span = self.y_max - self.y_min
        if span <= 0:
            return self.bottom
        fraction = (float(value) - self.y_min) / span
        return self.bottom - fraction * (self.bottom - self.top)
    # def

    def data_y(self, pixel):
        """Pixel back to data. The inverse of y(), for reading a plot back."""
        span = self.bottom - self.top
        if span <= 0:
            return self.y_min
        fraction = (self.bottom - float(pixel)) / span
        return self.y_min + fraction * (self.y_max - self.y_min)
    # def

    def point(self, x_value, y_value):
        return (self.x(x_value), self.y(y_value))
    # def
# class


def _padded(values, fraction=0.06):
    low, high = min(values), max(values)
    if high == low:
        high = low + 1.0
    pad = (high - low) * fraction
    return (low - pad, high + pad)
# def


def build_plot(series, order=2, tolerance_deg=0.5, width=760, height=460,
               mode=MODE_DEVIATION):
    """Everything to draw, in pixels, plus the numbers the caption needs.

    `series` is curve_series() output: {direction: [(pwm, mean, sd, n)]}.
    Returns None when there is nothing plottable, rather than an empty frame
    that looks like a measurement of zero.

    Two modes, and the default is not the obvious one. MODE_CURVE draws angle
    against PWM, which is the honest picture of the relation and completely
    useless for reading a band off: a 1 deg separation on a 62 deg axis is a
    line thickness. MODE_DEVIATION subtracts a reference fit taken across *both*
    directions and plots what is left, so the band fills the plot and localised
    structure is visible. The reference has to be common to both directions - if
    each were referenced to its own fit, each fit would absorb that direction's
    offset and the band would vanish, which is the one thing being measured.
    """
    usable = {d: rows for d, rows in series.items() if rows}
    if not usable:
        return None

    xs_all, ys_all = [], []
    for rows in usable.values():
        xs_all.extend(row[0] for row in rows)
        ys_all.extend(row[1] for row in rows)

    reference = None
    if mode == MODE_DEVIATION:
        reference = fit_polynomial(xs_all, ys_all, order)
        if not reference:
            return None
        ys_all = [y - evaluate_polynomial(reference, x)
                  for x, y in zip(xs_all, ys_all)]

    y_range = _padded(ys_all)
    if reference:
        # Keep the tramlines in frame even when nothing reaches them, so an
        # in-spec result looks in-spec rather than merely off the top.
        limit = max(abs(y_range[0]), abs(y_range[1]), tolerance_deg * 1.4)
        y_range = (-limit, limit)

    box = PlotBox(_padded(xs_all), y_range, width, height)

    polylines = []
    markers = []
    texts = []
    stats = {"order": order, "tolerance_deg": tolerance_deg, "fits": {}}

    for direction, role, fit_role in DIRECTIONS:
        rows = usable.get(direction)
        if not rows:
            continue

        xs = [row[0] for row in rows]
        measured = [row[1] for row in rows]
        ys = ([y - evaluate_polynomial(reference, x) for x, y in zip(xs, measured)]
              if reference else measured)

        polylines.append({"role": role, "dash": direction < 0,
                          "points": [box.point(x, y) for x, y in zip(xs, ys)]})
        markers.extend({"role": role, "point": box.point(x, y)}
                       for x, y in zip(xs, ys))

        fit = fit_polynomial(xs, measured, order)
        if not fit:
            continue

        if reference:
            smooth = [(x, evaluate_polynomial(fit, x)
                       - evaluate_polynomial(reference, x))
                      for x, _ in _sample_fit(fit, min(xs), max(xs))]
        else:
            smooth = _sample_fit(fit, min(xs), max(xs))

        polylines.append({"role": fit_role, "dash": False, "width": 2,
                          "points": [box.point(x, y) for x, y in smooth]})

        residuals = [y - evaluate_polynomial(fit, x)
                     for x, y in zip(xs, measured)]
        stats["fits"][direction] = {
            "rms_deg": rms(residuals),
            "max_deg": max(residuals, key=abs),
            "max_pwm": xs[residuals.index(max(residuals, key=abs))],
        }

    band = band_profile(usable)
    if band:
        magnitudes = [abs(value) for _pwm, value in band]
        worst = max(band, key=lambda pair: abs(pair[1]))
        stats["band"] = {
            "mean_deg": sum(magnitudes) / len(magnitudes),
            "max_deg": abs(worst[1]),
            "max_pwm": worst[0],
        }

    # The tramlines: in deviation mode they are the acceptance band about zero,
    # which is what makes them read as rails rather than as another pair of
    # curves to interpret.
    if reference:
        for sign in (1, -1):
            polylines.append({
                "role": ROLE_TRAM, "dash": True,
                "points": [box.point(box.x_min, sign * tolerance_deg),
                           box.point(box.x_max, sign * tolerance_deg)]})
        polylines.append({
            "role": ROLE_AXIS, "dash": False,
            "points": [box.point(box.x_min, 0.0), box.point(box.x_max, 0.0)]})

    stats["mode"] = mode
    axes, axis_texts = _axes(box, reference is not None)
    texts.extend(axis_texts)

    return {"box": box, "polylines": polylines, "markers": markers,
            "axes": axes, "texts": texts, "stats": stats,
            "width": width, "height": height}
# def


def _sample_fit(fit, x_min, x_max, steps=80):
    span = x_max - x_min
    return [(x_min + (span * i) / steps,
             evaluate_polynomial(fit, x_min + (span * i) / steps))
            for i in range(steps + 1)]
# def


def _axes(box, deviation=False):
    lines = [
        {"role": ROLE_AXIS, "points": [(box.left, box.top), (box.left, box.bottom)]},
        {"role": ROLE_AXIS, "points": [(box.left, box.bottom), (box.right, box.bottom)]},
    ]
    texts = []

    for value in nice_ticks(box.x_min, box.x_max):
        x = box.x(value)
        lines.append({"role": ROLE_AXIS,
                      "points": [(x, box.bottom), (x, box.bottom + 5)]})
        texts.append({"x": x, "y": box.bottom + 18, "anchor": "middle",
                      "text": "%d" % round(value)})

    for value in nice_ticks(box.y_min, box.y_max):
        y = box.y(value)
        lines.append({"role": ROLE_AXIS,
                      "points": [(box.left - 5, y), (box.left, y)]})
        texts.append({"x": box.left - 9, "y": y + 4, "anchor": "end",
                      "text": "%g" % value})

    texts.append({"x": (box.left + box.right) / 2.0, "y": box.bottom + 40,
                  "anchor": "middle", "text": "PWM us"})
    texts.append({"x": 14, "y": box.top + 10, "anchor": "start",
                  "text": "deviation deg" if deviation else "angle deg"})
    return lines, texts
# def


def to_svg(plot, title="Creep curve"):
    """The same geometry as a standalone file.

    Light background and explicit colours: this is a file to open, print and
    attach to a report, not a themed page, so it must not depend on whatever is
    behind it.
    """
    if not plot:
        return None

    width, height = plot["width"], plot["height"]
    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
        'viewBox="0 0 %d %d">' % (width, height, width, height),
        '<title>%s</title>' % _escape(title),
        '<rect width="%d" height="%d" fill="#ffffff"/>' % (width, height),
    ]

    for line in plot["axes"]:
        parts.append(_polyline(line, SVG_COLOURS[ROLE_AXIS], 1.0))

    for line in plot["polylines"]:
        colour = SVG_COLOURS.get(line["role"], "#000000")
        parts.append(_polyline(line, colour, line.get("width", 1.5)))

    for marker in plot["markers"]:
        x, y = marker["point"]
        parts.append('<circle cx="%.2f" cy="%.2f" r="2.5" fill="%s"/>'
                     % (x, y, SVG_COLOURS.get(marker["role"], "#000000")))

    for text in plot["texts"]:
        parts.append(
            '<text x="%.2f" y="%.2f" text-anchor="%s" font-family="sans-serif" '
            'font-size="11" fill="%s">%s</text>'
            % (text["x"], text["y"], text["anchor"], SVG_COLOURS[ROLE_AXIS],
               _escape(text["text"])))

    parts.append('</svg>')
    return "\n".join(parts)
# def


def _polyline(line, colour, width):
    points = " ".join("%.2f,%.2f" % (x, y) for x, y in line["points"])
    dash = ' stroke-dasharray="5 4"' if line.get("dash") else ""
    return ('<polyline points="%s" fill="none" stroke="%s" stroke-width="%.1f"%s/>'
            % (points, colour, width, dash))
# def


def _escape(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))
# def
