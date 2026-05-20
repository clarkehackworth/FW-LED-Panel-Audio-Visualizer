#!/usr/bin/env python3
"""
Patch inputmodule-rs to add SetEQConfig (0x1D) and DisplayEQ (0x21) commands,
with a per-bar brightness gradient (bright at base, configurable fade at tip).

Run from the root of the cloned inputmodule-rs repository:
    python3 /path/to/patch.py

The script is idempotent — each patch has a unique marker string that is only
present in the new content, never in the original source.
"""

import sys
from pathlib import Path

CONTROL  = Path("fl16-inputmodules/src/control.rs")
MATRIX   = Path("fl16-inputmodules/src/matrix.rs")
PATTERNS = Path("fl16-inputmodules/src/patterns.rs")
MAIN     = Path("ledmatrix/src/main.rs")

for f in (CONTROL, MATRIX, PATTERNS, MAIN):
    if not f.exists():
        print(f"ERROR: {f} not found — run from the root of inputmodule-rs")
        sys.exit(1)


def patch(path: Path, old, new: str, unique_marker: str, description: str) -> None:
    """
    Replace `old` with `new` in `path`.

    `old` may be a str or a list of str; each is tried in order and the first
    match wins.  This lets a single patch handle both the pristine source and
    an older patched version that needs upgrading.

    `unique_marker` must be present in `new` but absent from every `old` and
    from the original source, so we can detect whether the patch was already
    applied.
    """
    content = path.read_text(encoding="utf-8")
    if unique_marker in content:
        print(f"  SKIP (already applied): {description}")
        return
    olds = [old] if isinstance(old, str) else list(old)
    for o in olds:
        if o in content:
            path.write_text(content.replace(o, new, 1), encoding="utf-8")
            print(f"  OK: {description}")
            return
    print(f"  ERROR: anchor not found in {path}")
    print(f"         Looking for: {olds[0]!r}")
    sys.exit(1)


# ---------------------------------------------------------------------------
print(f"Patching {CONTROL} ...")
# ---------------------------------------------------------------------------

# 1. CommandVals enum — add SetEQConfig/DisplayEQ opcodes
#    Pristine upstream already has PwmFreq/DebugMode/Version; we insert
#    SetEQConfig (0x1D) before PwmFreq and DisplayEQ (0x21) after Version.
patch(
    CONTROL,
    old='    AnimationPeriod = 0x1C,\n    PwmFreq = 0x1E,\n    DebugMode = 0x1F,\n    Version = 0x20,',
    new='''\
    AnimationPeriod = 0x1C,
    // EQ visualizer — direction stored on-device, bar/peak data sent each frame
    SetEQConfig = 0x1D,
    PwmFreq = 0x1E,
    DebugMode = 0x1F,
    Version = 0x20,
    DisplayEQ = 0x21,''',
    unique_marker='SetEQConfig = 0x1D,',
    description="Add SetEQConfig/DisplayEQ to CommandVals enum",
)

# 2. Command enum — insert gated variants after DrawGreyColBuffer
#    Handles pristine source and old patched versions.
patch(
    CONTROL,
    old=[
        # Upgrade path: current patch — has SetEQConfig struct but DisplayEQ without peak_brightness
        '''\
    DrawGreyColBuffer,
    #[cfg(feature = "ledmatrix")]
    SetEQConfig { flags: u8, fade_min: u8 },
    #[cfg(feature = "ledmatrix")]
    DisplayEQ { heights: [u8; 34], peaks: [u8; 34] },
    #[cfg(feature = "b1display")]''',
        # Upgrade path: old patch used tuple variant SetEQConfig(u8)
        '''\
    DrawGreyColBuffer,
    #[cfg(feature = "ledmatrix")]
    SetEQConfig(u8),
    #[cfg(feature = "ledmatrix")]
    DisplayEQ { heights: [u8; 34], peaks: [u8; 34] },
    #[cfg(feature = "b1display")]''',
        # Upgrade path: even older patch used [u8; 9]
        '''\
    DrawGreyColBuffer,
    #[cfg(feature = "ledmatrix")]
    SetEQConfig(u8),
    #[cfg(feature = "ledmatrix")]
    DisplayEQ { heights: [u8; 9], peaks: [u8; 9] },
    #[cfg(feature = "b1display")]''',
        # Fresh path: pristine source
        '    DrawGreyColBuffer,\n    #[cfg(feature = "b1display")]',
    ],
    new='''\
    DrawGreyColBuffer,
    #[cfg(feature = "ledmatrix")]
    SetEQConfig { flags: u8, fade_min: u8 },
    #[cfg(feature = "ledmatrix")]
    DisplayEQ { heights: [u8; 34], peaks: [u8; 34], peak_brightness: [u8; 34] },
    #[cfg(feature = "b1display")]''',
    unique_marker='peak_brightness: [u8; 34]',
    description="Add SetEQConfig/DisplayEQ variants to Command enum",
)

# 3. Parsing — insert/upgrade arms before DrawGreyColBuffer
patch(
    CONTROL,
    old=[
        # Upgrade path: current patch — has fade_min but DisplayEQ without peak_brightness
        '''\
            Some(CommandVals::SetEQConfig) => {
                if count >= 4 {
                    let fade_min = if count >= 5 { buf[4] } else { 60 };
                    Some(Command::SetEQConfig { flags: buf[3], fade_min })
                } else {
                    None
                }
            }
            Some(CommandVals::DisplayEQ) => {
                // Payload: 17 bytes heights + 17 bytes peaks, nibble-packed (low nibble first).
                // Total packet = 3 header + 34 payload = 37 bytes (fits in 64-byte USB buffer).
                if count >= 37 {
                    let mut heights = [0u8; 34];
                    let mut peaks   = [0u8; 34];
                    for i in 0..17 {
                        heights[i * 2]     =  buf[3 + i]        & 0x0F;
                        heights[i * 2 + 1] = (buf[3 + i] >> 4)  & 0x0F;
                        peaks[i * 2]       =  buf[20 + i]        & 0x0F;
                        peaks[i * 2 + 1]   = (buf[20 + i] >> 4) & 0x0F;
                    }
                    Some(Command::DisplayEQ { heights, peaks })
                } else {
                    None
                }
            }
            Some(CommandVals::DrawGreyColBuffer) => Some(Command::DrawGreyColBuffer),''',
        # Upgrade path: old patch used SetEQConfig(buf[3]) without fade_min
        '''\
            Some(CommandVals::SetEQConfig) => {
                if count >= 4 {
                    Some(Command::SetEQConfig(buf[3]))
                } else {
                    None
                }
            }
            Some(CommandVals::DisplayEQ) => {
                // Payload: 17 bytes heights + 17 bytes peaks, nibble-packed (low nibble first).
                // Total packet = 3 header + 34 payload = 37 bytes (fits in 64-byte USB buffer).
                if count >= 37 {
                    let mut heights = [0u8; 34];
                    let mut peaks   = [0u8; 34];
                    for i in 0..17 {
                        heights[i * 2]     =  buf[3 + i]        & 0x0F;
                        heights[i * 2 + 1] = (buf[3 + i] >> 4)  & 0x0F;
                        peaks[i * 2]       =  buf[20 + i]        & 0x0F;
                        peaks[i * 2 + 1]   = (buf[20 + i] >> 4) & 0x0F;
                    }
                    Some(Command::DisplayEQ { heights, peaks })
                } else {
                    None
                }
            }
            Some(CommandVals::DrawGreyColBuffer) => Some(Command::DrawGreyColBuffer),''',
        # Upgrade path: raw 34-byte layout (exceeds 64-byte USB packet limit)
        '''\
            Some(CommandVals::SetEQConfig) => {
                if count >= 4 {
                    Some(Command::SetEQConfig(buf[3]))
                } else {
                    None
                }
            }
            Some(CommandVals::DisplayEQ) => {
                if count >= 71 {
                    let mut heights = [0u8; 34];
                    let mut peaks = [0u8; 34];
                    heights.copy_from_slice(&buf[3..37]);
                    peaks.copy_from_slice(&buf[37..71]);
                    Some(Command::DisplayEQ { heights, peaks })
                } else {
                    None
                }
            }
            Some(CommandVals::DrawGreyColBuffer) => Some(Command::DrawGreyColBuffer),''',
        # Upgrade path: old [u8; 9] parsing block
        '''\
            Some(CommandVals::SetEQConfig) => {
                if count >= 4 {
                    Some(Command::SetEQConfig(buf[3]))
                } else {
                    None
                }
            }
            Some(CommandVals::DisplayEQ) => {
                if count >= 21 {
                    let mut heights = [0u8; 9];
                    let mut peaks = [0u8; 9];
                    heights.copy_from_slice(&buf[3..12]);
                    peaks.copy_from_slice(&buf[12..21]);
                    Some(Command::DisplayEQ { heights, peaks })
                } else {
                    None
                }
            }
            Some(CommandVals::DrawGreyColBuffer) => Some(Command::DrawGreyColBuffer),''',
        # Fresh path: pristine source
        '            Some(CommandVals::DrawGreyColBuffer) => Some(Command::DrawGreyColBuffer),',
    ],
    new='''\
            Some(CommandVals::SetEQConfig) => {
                if count >= 4 {
                    let fade_min = if count >= 5 { buf[4] } else { 60 };
                    Some(Command::SetEQConfig { flags: buf[3], fade_min })
                } else {
                    None
                }
            }
            Some(CommandVals::DisplayEQ) => {
                // Payload: 17B heights + 17B peaks + 17B peak_brightness, nibble-packed.
                // 3 header + 51 payload = 54 bytes (fits in 64-byte USB buffer).
                // peak_brightness is optional; defaults to 255 if not sent.
                if count >= 37 {
                    let mut heights = [0u8; 34];
                    let mut peaks   = [0u8; 34];
                    let mut peak_brightness = [255u8; 34];
                    for i in 0..17 {
                        heights[i * 2]     =  buf[3 + i]        & 0x0F;
                        heights[i * 2 + 1] = (buf[3 + i] >> 4)  & 0x0F;
                        peaks[i * 2]       =  buf[20 + i]        & 0x0F;
                        peaks[i * 2 + 1]   = (buf[20 + i] >> 4) & 0x0F;
                    }
                    if count >= 54 {
                        for i in 0..17 {
                            let lo = buf[37 + i] & 0x0F;
                            let hi = (buf[37 + i] >> 4) & 0x0F;
                            peak_brightness[i * 2]     = lo * 17;
                            peak_brightness[i * 2 + 1] = hi * 17;
                        }
                    }
                    Some(Command::DisplayEQ { heights, peaks, peak_brightness })
                } else {
                    None
                }
            }
            Some(CommandVals::DrawGreyColBuffer) => Some(Command::DrawGreyColBuffer),''',
    unique_marker='peak_brightness[i * 2]     = lo * 17;',
    description="Add parsing arms for SetEQConfig and DisplayEQ",
)

# 4. Handler — insert before the existing DrawGreyColBuffer handler
patch(
    CONTROL,
    old=[
        # Upgrade path: current patch — handler passes heights/peaks but not peak_brightness
        '''\
        Command::SetEQConfig { flags, fade_min } => {
            state.eq_direction = *flags;
            state.eq_fade_min = *fade_min;
            None
        }
        Command::DisplayEQ { heights, peaks } => {
            state.grid = render_eq_bars(heights, peaks, state.eq_direction, state.eq_fade_min);
            state.animate = false;
            None
        }
        Command::DrawGreyColBuffer => {''',
        # Upgrade path: old handler used tuple variant
        '''\
        Command::SetEQConfig(direction) => {
            state.eq_direction = *direction;
            None
        }
        Command::DisplayEQ { heights, peaks } => {
            state.grid = render_eq_bars(heights, peaks, state.eq_direction);
            state.animate = false;
            None
        }
        Command::DrawGreyColBuffer => {''',
        # Fresh path: pristine source
        '        Command::DrawGreyColBuffer => {',
    ],
    new='''\
        Command::SetEQConfig { flags, fade_min } => {
            state.eq_direction = *flags;
            state.eq_fade_min = *fade_min;
            None
        }
        Command::DisplayEQ { heights, peaks, peak_brightness } => {
            state.grid = render_eq_bars(heights, peaks, state.eq_direction, state.eq_fade_min, peak_brightness);
            state.animate = false;
            None
        }
        Command::DrawGreyColBuffer => {''',
    unique_marker='render_eq_bars(heights, peaks, state.eq_direction, state.eq_fade_min, peak_brightness)',
    description="Add handlers for SetEQConfig and DisplayEQ",
)

# ---------------------------------------------------------------------------
print(f"Patching {MATRIX} ...")
# ---------------------------------------------------------------------------

# 5. LedmatrixState — add eq_direction and eq_fade_min fields after brightness
patch(
    MATRIX,
    old=[
        # Upgrade path: old patch added eq_direction but not eq_fade_min
        '''\
    /// LED brightness out of 255
    pub brightness: u8,
    /// Bar direction for DisplayEQ: 0 = base at col 0, grows toward col 8 (right panel); 1 = base at col 8, grows toward col 0 (left panel, mirrored)
    pub eq_direction: u8,''',
        # Fresh path: pristine source
        '    /// LED brightness out of 255\n    pub brightness: u8,',
    ],
    new='''\
    /// LED brightness out of 255
    pub brightness: u8,
    /// Bar direction for DisplayEQ: 0 = base at col 0, grows toward col 8 (right panel); 1 = base at col 8, grows toward col 0 (left panel, mirrored)
    pub eq_direction: u8,
    /// Brightness at the tip of each EQ bar (0-255); base is always 255
    pub eq_fade_min: u8,''',
    unique_marker='pub eq_fade_min: u8,',
    description="Add eq_direction and eq_fade_min fields to LedmatrixState",
)

# ---------------------------------------------------------------------------
print(f"Patching {MAIN} ...")
# ---------------------------------------------------------------------------

# 6. LedmatrixState initializer — add eq_direction and eq_fade_min
patch(
    MAIN,
    old=[
        # Upgrade path: old patch added eq_direction but not eq_fade_min
        '        eq_direction: 0,\n        sleeping:',
        # Fresh path: pristine source
        '        sleeping:',
    ],
    new='''\
        eq_direction: 0,
        eq_fade_min: 60,
        sleeping:''',
    unique_marker='eq_fade_min: 60,',
    description="Initialize eq_direction and eq_fade_min in LedmatrixState",
)

# ---------------------------------------------------------------------------
print(f"Patching {PATTERNS} ...")
# ---------------------------------------------------------------------------

# 7. render_eq_bars — insert or upgrade the function at end of file.
#    Portrait layout: 34 bands along firmware row axis (HEIGHT=34),
#    bars along firmware col axis (WIDTH=9), up to 9 pixels.

_EQ_OLD_9 = r"""
/// Render frequency-bar graph into a Grid.
///
/// heights: bar height per column (0-33 pixels)
/// peaks:   peak-indicator position per column (0 = none, 1-33 = pixel height above base)
/// flags bit 0: 0 = bars extend right (row 0 = base, grow toward row HEIGHT-1), 1 = left (row HEIGHT-1 = base)
/// flags bit 1: 0 = band[0] at col 0 (top), 1 = band[0] at col WIDTH-1 (bottom)
pub fn render_eq_bars(heights: &[u8; 9], peaks: &[u8; 9], flags: u8) -> Grid {
    use crate::matrix::{HEIGHT, WIDTH};
    let mut grid = Grid([[0u8; HEIGHT]; WIDTH]);
    let rtl = flags & 0x01 != 0;
    let freq_reversed = flags & 0x02 != 0;

    for i in 0..WIDTH {
        let col = if freq_reversed { WIDTH - 1 - i } else { i };
        let h = (heights[i] as usize).min(HEIGHT);
        let p = peaks[i] as usize;

        if !rtl {
            for row in 0..h {
                grid.0[col][row] = 180;
            }
            if p > 0 && p <= HEIGHT {
                grid.0[col][p - 1] = 255;
            }
        } else {
            for row in (HEIGHT - h)..HEIGHT {
                grid.0[col][row] = 180;
            }
            if p > 0 && p <= HEIGHT {
                grid.0[col][HEIGHT - p] = 255;
            }
        }
    }

    grid
}
"""

_EQ_OLD_34_FLAT = r"""
/// Render a portrait-mode frequency-bar graph into a Grid.
///
/// 34 frequency bands, one per firmware row (row 0-33, physical right->left).
/// Bars grow along the firmware column axis (col 0-8, physical top->bottom),
/// up to 9 pixels long.
///
/// heights:     bar length per band in cols, 0-9   (34 bands, one per row)
/// peaks:       peak-dot col offset per band (0 = none, 1-9 from bar base)
/// flags bit 0: 0 = base at col 0, grows toward col 8  (right panel)
///              1 = base at col 8, grows toward col 0  (left panel, mirrored)
/// flags bit 1: 0 = band[0] at row 0  |  1 = band[0] at row HEIGHT-1 (reversed)
pub fn render_eq_bars(heights: &[u8; 34], peaks: &[u8; 34], flags: u8) -> Grid {
    use crate::matrix::{HEIGHT, WIDTH};
    let mut grid = Grid([[0u8; HEIGHT]; WIDTH]);
    let from_high = flags & 0x01 != 0;
    let freq_rev   = flags & 0x02 != 0;

    for i in 0..HEIGHT {
        let row = if freq_rev { HEIGHT - 1 - i } else { i };
        let h = (heights[i] as usize).min(WIDTH);
        let p = peaks[i] as usize;

        if !from_high {
            for col in 0..h {
                grid.0[col][row] = 180;
            }
            if p > 0 && p <= WIDTH {
                grid.0[p - 1][row] = 255;
            }
        } else {
            for col in (WIDTH - h)..WIDTH {
                grid.0[col][row] = 180;
            }
            if p > 0 && p <= WIDTH {
                grid.0[WIDTH - p][row] = 255;
            }
        }
    }

    grid
}
"""

_EQ_OLD_GRADIENT = r"""
/// Linear brightness gradient: full at the bar base, dimmer at the tip.
fn bar_brightness(dist_from_base: usize, bar_len: usize, fade_min: u8) -> u8 {
    if bar_len <= 1 {
        return 255;
    }
    let min = fade_min as u32;
    let t = dist_from_base as u32 * (255 - min) / (bar_len as u32 - 1);
    (255 - t) as u8
}

/// Render a portrait-mode frequency-bar graph into a Grid.
///
/// 34 frequency bands, one per firmware row (row 0-33, physical right->left).
/// Bars grow along the firmware column axis (col 0-8, physical top->bottom),
/// up to 9 pixels long.
///
/// heights:     bar length per band in cols, 0-9   (34 bands, one per row)
/// peaks:       peak-dot col offset per band (0 = none, 1-9 from bar base)
/// fade_min:    brightness at the tip of each bar (0-255); base is always 255
/// flags bit 0: 0 = base at col 0, grows toward col 8  (right panel)
///              1 = base at col 8, grows toward col 0  (left panel, mirrored)
/// flags bit 1: 0 = band[0] at row 0  |  1 = band[0] at row HEIGHT-1 (reversed)
pub fn render_eq_bars(heights: &[u8; 34], peaks: &[u8; 34], flags: u8, fade_min: u8) -> Grid {
    use crate::matrix::{HEIGHT, WIDTH};
    let mut grid = Grid([[0u8; HEIGHT]; WIDTH]);
    let from_high = flags & 0x01 != 0;
    let freq_rev   = flags & 0x02 != 0;

    for i in 0..HEIGHT {
        let row = if freq_rev { HEIGHT - 1 - i } else { i };
        let h = (heights[i] as usize).min(WIDTH);
        let p = peaks[i] as usize;

        if !from_high {
            // base at col 0, tip at col h-1
            for col in 0..h {
                grid.0[col][row] = bar_brightness(col, h, fade_min);
            }
            if p > 0 && p <= WIDTH {
                grid.0[p - 1][row] = 255;
            }
        } else {
            // base at col WIDTH-1, tip at col WIDTH-h
            for col in (WIDTH - h)..WIDTH {
                let dist = (WIDTH - 1) - col;
                grid.0[col][row] = bar_brightness(dist, h, fade_min);
            }
            if p > 0 && p <= WIDTH {
                grid.0[WIDTH - p][row] = 255;
            }
        }
    }

    grid
}
"""

_EQ_NEW = r"""
/// Linear brightness gradient: full at the bar base, dimmer at the tip.
fn bar_brightness(dist_from_base: usize, bar_len: usize, fade_min: u8) -> u8 {
    if bar_len <= 1 {
        return 255;
    }
    let min = fade_min as u32;
    let t = dist_from_base as u32 * (255 - min) / (bar_len as u32 - 1);
    (255 - t) as u8
}

/// Render a portrait-mode frequency-bar graph into a Grid.
///
/// 34 frequency bands, one per firmware row (row 0-33, physical right->left).
/// Bars grow along the firmware column axis (col 0-8, physical top->bottom),
/// up to 9 pixels long.
///
/// heights:          bar length per band in cols, 0-9   (34 bands, one per row)
/// peaks:            peak-dot col offset per band (0 = none, 1-9 from bar base)
/// peak_brightness:  brightness of each peak dot (0-255); fades as peak holds/falls
/// fade_min:         brightness at the tip of each bar (0-255); base is always 255
/// flags bit 0: 0 = base at col 0, grows toward col 8  (right panel)
///              1 = base at col 8, grows toward col 0  (left panel, mirrored)
/// flags bit 1: 0 = band[0] at row 0  |  1 = band[0] at row HEIGHT-1 (reversed)
pub fn render_eq_bars(heights: &[u8; 34], peaks: &[u8; 34], flags: u8, fade_min: u8, peak_brightness: &[u8; 34]) -> Grid {
    use crate::matrix::{HEIGHT, WIDTH};
    let mut grid = Grid([[0u8; HEIGHT]; WIDTH]);
    let from_high = flags & 0x01 != 0;
    let freq_rev   = flags & 0x02 != 0;

    for i in 0..HEIGHT {
        let row = if freq_rev { HEIGHT - 1 - i } else { i };
        let h = (heights[i] as usize).min(WIDTH);
        let p = peaks[i] as usize;

        if !from_high {
            // base at col 0, tip at col h-1
            for col in 0..h {
                grid.0[col][row] = bar_brightness(col, h, fade_min);
            }
            if p > 0 && p <= WIDTH {
                grid.0[p - 1][row] = peak_brightness[i];
            }
        } else {
            // base at col WIDTH-1, tip at col WIDTH-h
            for col in (WIDTH - h)..WIDTH {
                let dist = (WIDTH - 1) - col;
                grid.0[col][row] = bar_brightness(dist, h, fade_min);
            }
            if p > 0 && p <= WIDTH {
                grid.0[WIDTH - p][row] = peak_brightness[i];
            }
        }
    }

    grid
}
"""

_GRADIENT_MARKER = 'peak_brightness: &[u8; 34]'
content = PATTERNS.read_text(encoding="utf-8")
if _GRADIENT_MARKER in content:
    print("  SKIP (already applied): render_eq_bars gradient + peak_brightness")
elif _EQ_OLD_GRADIENT in content:
    PATTERNS.write_text(content.replace(_EQ_OLD_GRADIENT, _EQ_NEW, 1), encoding="utf-8")
    print("  OK: render_eq_bars (upgraded gradient → peak_brightness)")
elif _EQ_OLD_34_FLAT in content:
    PATTERNS.write_text(content.replace(_EQ_OLD_34_FLAT, _EQ_NEW, 1), encoding="utf-8")
    print("  OK: render_eq_bars (upgraded flat-34 → gradient + peak_brightness)")
elif _EQ_OLD_9 in content:
    PATTERNS.write_text(content.replace(_EQ_OLD_9, _EQ_NEW, 1), encoding="utf-8")
    print("  OK: render_eq_bars (upgraded [u8;9] → gradient + peak_brightness)")
else:
    PATTERNS.write_text(content + _EQ_NEW, encoding="utf-8")
    print("  OK: render_eq_bars (appended gradient + peak_brightness)")

# ---------------------------------------------------------------------------
print()
print("Patch complete.")
print()
print("Verify with:  git diff fl16-inputmodules/src/ ledmatrix/src/")
print("Build with:   cargo build --release -p ledmatrix")
