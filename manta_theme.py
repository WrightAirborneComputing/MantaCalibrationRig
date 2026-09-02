"""Visual theme for the Manta Trimmer window.

One place that owns fonts, colours and widget styling, so restyling the app is
editing this file rather than hunting for a hard-coded "dark green" in the
middle of a MAVLink callback.

Two mechanisms, because the app uses both widget sets:

  ttk.Style      - for the ttk widgets (notebook, combobox, progressbar,
                   treeview) and for the named styles the layout can opt into,
                   e.g. Primary.TButton.
  option_add     - defaults for the classic tk widgets the app is mostly built
                   from today. These are added at the default "interactive"
                   priority, so an explicit bg= or relief= passed at
                   construction still wins. That is deliberate: this file
                   changes how unstyled widgets look without overriding a
                   deliberate choice made at the call site.

Depends on tkinter and nothing else, and touches no layout - importing it does
nothing until apply_theme() is called.
"""

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk


# A bench instrument, not a document: cool neutrals with a slight blue-green
# bias so the teal accent sits on them without fighting. The three semantic
# colours are separate from the accent - they mean pass, watch and fail, and
# using the accent for any of them would make "running" look like "good".
PALETTE = {
    "paper":     "#e9edf0",   # window and tab background
    "panel":     "#ffffff",   # raised surfaces: cards, entry fields, canvases
    "panel_alt": "#f3f6f8",   # button faces, troughs, inert fills
    "ink":       "#16222b",   # primary text
    "ink_muted": "#495c68",   # secondary text
    "ink_faint": "#7b8d98",   # labels, units, disabled text
    "rule":      "#c9d4da",   # borders
    "rule_soft": "#dde5ea",   # internal dividers
    "accent":    "#0d6f77",   # the one accent: primary action, live values
    "accent_lo": "#d6ebec",   # accent wash for selection and hover
    "ok":        "#2c7350",   # connected, calibrated, pass
    "warn":      "#9a6206",   # busy, non-default state, needs a look
    "bad":       "#a83a2d",   # disconnected, failed, stop
}

# Preference order, first hit wins. The rig runs on Linux boxes and the odd
# Windows laptop, and Tk silently substitutes a family it does not have - which
# is how you end up shipping a font you never looked at.
UI_FONT_CANDIDATES = (
    "Inter", "Cantarell", "Ubuntu", "Segoe UI", "Noto Sans", "DejaVu Sans",
)
MONO_FONT_CANDIDATES = (
    "JetBrains Mono", "Source Code Pro", "Ubuntu Mono", "Consolas",
    "DejaVu Sans Mono",
)

BASE_SIZE = 10
SMALL_SIZE = 9
READOUT_SIZE = 22


def _pick_family(root, candidates, fallback):
    """First candidate the running Tk actually has installed."""
    try:
        available = set(tkfont.families(root))
    except tk.TclError:
        return fallback
    for name in candidates:
        if name in available:
            return name
    return fallback
# def


def _apply_fonts(root):
    """Restyle the named fonts, which restyles every widget that inherits them.

    Tk's widgets default to TkDefaultFont and friends, so setting these here is
    what saves passing font= to several hundred widget constructors.
    """
    ui = _pick_family(root, UI_FONT_CANDIDATES, "TkDefaultFont")
    mono = _pick_family(root, MONO_FONT_CANDIDATES, "TkFixedFont")

    spec = {
        "TkDefaultFont":      (ui, BASE_SIZE, "normal"),
        "TkTextFont":         (ui, BASE_SIZE, "normal"),
        "TkMenuFont":         (ui, BASE_SIZE, "normal"),
        "TkHeadingFont":      (ui, BASE_SIZE, "bold"),
        "TkCaptionFont":      (ui, BASE_SIZE, "bold"),
        "TkSmallCaptionFont": (ui, SMALL_SIZE, "normal"),
        "TkTooltipFont":      (ui, SMALL_SIZE, "normal"),
        "TkIconFont":         (ui, BASE_SIZE, "normal"),
        "TkFixedFont":        (mono, SMALL_SIZE, "normal"),
    }

    for name, (family, size, weight) in spec.items():
        try:
            handle = tkfont.nametofont(name, root=root)
        except tk.TclError:
            # Not every platform defines every named font.
            continue
        handle.configure(family=family, size=size, weight=weight)
    # for

    return ui, mono
# def


def _apply_ttk_styles(root, ui_family, mono_family):
    """Style the ttk widgets, and define the named styles layout can ask for.

    clam is the only stock theme that honours background and bordercolor on
    most elements - default and alt draw their own bevels and ignore much of
    what you configure, which is exactly the 1990s look being removed here.
    """
    style = ttk.Style(root)

    try:
        style.theme_use("clam")
    except tk.TclError:
        # Windows-only builds may not carry clam; the fonts and the option
        # database still apply, so this is a degraded result, not a failure.
        return style

    p = PALETTE

    # The root style. Everything below overrides only what it needs to.
    style.configure(
        ".",
        background=p["paper"],
        foreground=p["ink"],
        fieldbackground=p["panel"],
        bordercolor=p["rule"],
        lightcolor=p["rule_soft"],
        darkcolor=p["rule_soft"],
        troughcolor=p["rule_soft"],
        focuscolor=p["accent"],
        selectbackground=p["accent"],
        selectforeground=p["panel"],
        borderwidth=1,
        relief="flat",
    )
    style.map(
        ".",
        foreground=[("disabled", p["ink_faint"])],
    )

    style.configure("TFrame", background=p["paper"])
    style.configure("TLabel", background=p["paper"], foreground=p["ink"])

    style.configure(
        "TLabelframe",
        background=p["paper"],
        bordercolor=p["rule"],
        borderwidth=1,
        relief="solid",
    )
    style.configure(
        "TLabelframe.Label",
        background=p["paper"],
        foreground=p["ink_faint"],
        font=(ui_family, SMALL_SIZE, "bold"),
    )

    style.configure(
        "TButton",
        background=p["paper"],
        foreground=p["ink"],
        bordercolor=p["rule"],
        lightcolor=p["paper"],
        darkcolor=p["paper"],
        relief="flat",
        padding=(10, 5),
        anchor="center",
    )
    style.map(
        "TButton",
        background=[("disabled", p["panel_alt"]), ("pressed", p["rule_soft"]),
                    ("active", p["accent_lo"])],
        bordercolor=[("focus", p["accent"]), ("active", p["accent"])],
        foreground=[("disabled", p["ink_faint"])],
    )

    style.configure(
        "TEntry",
        fieldbackground=p["panel"],
        foreground=p["ink"],
        bordercolor=p["rule"],
        lightcolor=p["rule"],
        darkcolor=p["rule"],
        insertcolor=p["ink"],
        padding=(6, 4),
        relief="flat",
    )
    style.map(
        "TEntry",
        bordercolor=[("focus", p["accent"])],
        lightcolor=[("focus", p["accent"])],
        darkcolor=[("focus", p["accent"])],
        fieldbackground=[("disabled", p["paper"])],
    )

    style.configure(
        "TCombobox",
        fieldbackground=p["panel"],
        background=p["panel_alt"],
        foreground=p["ink"],
        bordercolor=p["rule"],
        lightcolor=p["rule"],
        darkcolor=p["rule"],
        arrowcolor=p["ink_muted"],
        padding=(6, 4),
    )
    style.map(
        "TCombobox",
        bordercolor=[("focus", p["accent"])],
        arrowcolor=[("active", p["accent"])],
        fieldbackground=[("disabled", p["paper"])],
    )
    # The dropdown list is a plain Tk listbox owned by Tcl, so it is reached
    # through the option database rather than through ttk.
    root.option_add("*TCombobox*Listbox.background", p["panel"])
    root.option_add("*TCombobox*Listbox.foreground", p["ink"])
    root.option_add("*TCombobox*Listbox.selectBackground", p["accent"])
    root.option_add("*TCombobox*Listbox.selectForeground", p["panel"])
    root.option_add("*TCombobox*Listbox.borderWidth", 0)

    style.configure("TCheckbutton", background=p["paper"], foreground=p["ink"])
    style.map(
        "TCheckbutton",
        indicatorcolor=[("selected", p["accent"])],
        background=[("active", p["paper"])],
    )
    style.configure("TRadiobutton", background=p["paper"], foreground=p["ink"])
    style.map(
        "TRadiobutton",
        indicatorcolor=[("selected", p["accent"])],
        background=[("active", p["paper"])],
    )

    style.configure("TNotebook", background=p["paper"], borderwidth=0,
                    tabmargins=(0, 4, 0, 0))
    style.configure(
        "TNotebook.Tab",
        background=p["paper"],
        foreground=p["ink_faint"],
        bordercolor=p["paper"],
        lightcolor=p["paper"],
        padding=(14, 7),
    )
    style.map(
        "TNotebook.Tab",
        background=[("selected", p["panel"])],
        foreground=[("selected", p["ink"]), ("active", p["ink_muted"])],
        lightcolor=[("selected", p["panel"])],
        bordercolor=[("selected", p["rule"])],
    )

    style.configure(
        "TProgressbar",
        background=p["accent"],
        troughcolor=p["rule_soft"],
        bordercolor=p["rule_soft"],
        lightcolor=p["accent"],
        darkcolor=p["accent"],
        thickness=8,
    )

    style.configure(
        "Treeview",
        background=p["panel"],
        fieldbackground=p["panel"],
        foreground=p["ink"],
        bordercolor=p["rule"],
        rowheight=22,
        relief="flat",
    )
    style.map(
        "Treeview",
        background=[("selected", p["accent_lo"])],
        foreground=[("selected", p["ink"])],
    )
    style.configure(
        "Treeview.Heading",
        background=p["paper"],
        foreground=p["ink_faint"],
        font=(ui_family, SMALL_SIZE, "bold"),
        relief="flat",
        padding=(6, 4),
    )
    style.map("Treeview.Heading", background=[("active", p["rule_soft"])])

    style.configure(
        "TScrollbar",
        background=p["panel_alt"],
        troughcolor=p["paper"],
        bordercolor=p["paper"],
        arrowcolor=p["ink_faint"],
        relief="flat",
    )
    style.map("TScrollbar", background=[("active", p["rule"])])

    # ---- named styles the layout can opt into --------------------------------

    # A raised surface. Sits on paper, so it reads as a group without needing
    # the bevelled border every panel currently draws.
    style.configure("Card.TFrame", background=p["panel"], relief="flat",
                    borderwidth=0)
    style.configure("Card.TLabel", background=p["panel"], foreground=p["ink"])

    # A measured value, sized to be read across a bench. Monospaced so the
    # digits do not shuffle sideways as the angle changes.
    style.configure(
        "Readout.TLabel",
        background=p["panel"],
        foreground=p["accent"],
        font=(mono_family, READOUT_SIZE, "bold"),
        anchor="center",
    )
    style.configure(
        "Units.TLabel",
        background=p["panel"],
        foreground=p["ink_faint"],
        font=(ui_family, SMALL_SIZE, "normal"),
    )
    style.configure("Muted.TLabel", foreground=p["ink_faint"])
    style.configure("Heading.TLabel", foreground=p["ink_faint"],
                    font=(ui_family, SMALL_SIZE, "bold"))

    # One primary action per panel. Spending the accent anywhere else is what
    # makes a window with fourteen buttons unreadable.
    style.configure(
        "Primary.TButton",
        background=p["accent"],
        foreground=p["panel"],
        bordercolor=p["accent"],
        lightcolor=p["accent"],
        darkcolor=p["accent"],
        font=(ui_family, BASE_SIZE, "bold"),
    )
    style.map(
        "Primary.TButton",
        background=[("disabled", p["rule_soft"]), ("pressed", p["ink"]),
                    ("active", p["ink"])],
        bordercolor=[("disabled", p["rule_soft"]), ("active", p["ink"])],
        foreground=[("disabled", p["ink_faint"])],
    )

    # Outlined rather than filled: stop is pressed in a hurry, but a wall of
    # red buttons is its own hazard.
    style.configure(
        "Danger.TButton",
        background=p["panel_alt"],
        foreground=p["bad"],
        bordercolor=p["bad"],
        lightcolor=p["panel_alt"],
        darkcolor=p["panel_alt"],
    )
    style.map(
        "Danger.TButton",
        background=[("disabled", p["paper"]), ("pressed", p["bad"]),
                    ("active", p["bad"])],
        foreground=[("disabled", p["ink_faint"]), ("pressed", p["panel"]),
                    ("active", p["panel"])],
        bordercolor=[("disabled", p["rule"])],
    )

    return style
# def


def _apply_classic_defaults(root, mono_family):
    """Defaults for the classic tk widgets, via the option database.

    The window is still mostly tk.Frame / tk.Label / tk.Button, which ttk
    styles do not reach. Setting them per class here is what lets the repaint
    land without rewriting the layout first.

    Priority is left at the default (interactive, 80): higher than a widget's
    built-in default, lower than an option passed at construction. So the
    relief="groove" rows still draw their grooves until they are removed - by
    design, so this file cannot silently undo a deliberate choice.
    """
    p = PALETTE
    opt = root.option_add

    # Two surfaces, not one. Content sits on panel (white) and the chrome
    # around it - the toplevel and the notebook - stays paper, so the tab body
    # reads as a page rather than as more window. Classic tk widgets do not
    # inherit their parent's background, so every container class that can end
    # up inside a tab has to be named here or it shows up as a grey patch.
    root.configure(background=p["paper"])

    for cls in ("Frame", "Labelframe", "Label", "Checkbutton", "Radiobutton",
                "Scale", "Message"):
        opt("*%s.background" % cls, p["panel"])
        opt("*%s.highlightThickness" % cls, 0)
    # for

    for cls in ("Label", "Checkbutton", "Radiobutton", "Message"):
        opt("*%s.foreground" % cls, p["ink"])
    # for

    opt("*Labelframe.foreground", p["ink_faint"])
    opt("*Labelframe.borderWidth", 1)
    opt("*Labelframe.relief", "solid")

    opt("*Checkbutton.activeBackground", p["panel"])
    opt("*Checkbutton.selectColor", p["panel"])
    opt("*Radiobutton.activeBackground", p["panel"])
    opt("*Radiobutton.selectColor", p["panel"])

    # The trim sliders. Stock Tk draws a bevelled trough with a chunky raised
    # thumb, which is the single most dated widget on the Trim tab.
    opt("*Scale.troughColor", p["rule"])
    opt("*Scale.activeBackground", p["accent"])
    opt("*Scale.foreground", p["ink_muted"])
    opt("*Scale.sliderRelief", "solid")
    opt("*Scale.borderWidth", 1)
    opt("*Scale.relief", "flat")
    opt("*Scale.sliderLength", 22)
    opt("*Scale.width", 12)

    opt("*Button.background", p["paper"])
    opt("*Button.foreground", p["ink"])
    opt("*Button.activeBackground", p["accent_lo"])
    opt("*Button.activeForeground", p["ink"])
    opt("*Button.disabledForeground", p["ink_faint"])
    opt("*Button.relief", "solid")
    opt("*Button.borderWidth", 1)
    opt("*Button.highlightThickness", 0)
    opt("*Button.padX", 10)
    opt("*Button.padY", 4)

    opt("*Entry.background", p["panel"])
    opt("*Entry.foreground", p["ink"])
    opt("*Entry.disabledBackground", p["paper"])
    opt("*Entry.disabledForeground", p["ink_faint"])
    opt("*Entry.readonlyBackground", p["paper"])
    opt("*Entry.insertBackground", p["ink"])
    opt("*Entry.selectBackground", p["accent"])
    opt("*Entry.selectForeground", p["panel"])
    opt("*Entry.relief", "flat")
    opt("*Entry.borderWidth", 1)
    # A 1px ring that turns accent on focus, which is cheaper than a border and
    # is the only focus cue a flat entry gets.
    opt("*Entry.highlightThickness", 1)
    opt("*Entry.highlightBackground", p["rule"])
    opt("*Entry.highlightColor", p["accent"])

    # The instrumentation log. Monospaced because it is aligned output, not
    # prose, and columns that drift are how a stray value gets missed.
    opt("*Text.background", p["panel"])
    opt("*Text.foreground", p["ink"])
    opt("*Text.insertBackground", p["ink"])
    opt("*Text.selectBackground", p["accent_lo"])
    opt("*Text.selectForeground", p["ink"])
    opt("*Text.font", (mono_family, SMALL_SIZE))
    opt("*Text.relief", "flat")
    opt("*Text.borderWidth", 0)
    opt("*Text.highlightThickness", 1)
    opt("*Text.highlightBackground", p["rule"])
    opt("*Text.highlightColor", p["rule"])

    opt("*Canvas.background", p["panel"])
    opt("*Canvas.highlightThickness", 1)
    opt("*Canvas.highlightBackground", p["rule"])

    opt("*Menu.background", p["panel"])
    opt("*Menu.foreground", p["ink"])
    opt("*Menu.activeBackground", p["accent"])
    opt("*Menu.activeForeground", p["panel"])
    opt("*Menu.relief", "flat")
    opt("*Menu.borderWidth", 1)

    opt("*Scrollbar.background", p["panel_alt"])
    opt("*Scrollbar.troughColor", p["paper"])
    opt("*Scrollbar.activeBackground", p["rule"])
    opt("*Scrollbar.relief", "flat")
    opt("*Scrollbar.borderWidth", 0)
    opt("*Scrollbar.highlightThickness", 0)
    opt("*Scrollbar.width", 12)
# def


def apply_theme(root):
    """Apply fonts, ttk styles and classic-widget defaults to a Tk instance.

    Call this before building any widgets: the option database is consulted at
    widget creation, so anything built earlier keeps the stock look.

    Returns the ttk.Style so a caller can extend it. Never raises on a Tk that
    is missing a theme or a named font - a rig with an unstyled window is
    usable, one that will not start is not.
    """
    ui_family, mono_family = _apply_fonts(root)
    style = _apply_ttk_styles(root, ui_family, mono_family)
    _apply_classic_defaults(root, mono_family)
    return style
# def


def status_colour(state):
    """Palette colour for a link or test state.

    Central so "connected" is the same green in the connection strip, the trim
    tab and the results table, and so changing it is one edit.

    state is one of "ok", "warn", "bad", or anything else for neutral.
    """
    return PALETTE.get(state, PALETTE["ink"])
# def
