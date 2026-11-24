"""
OV7670 viewer + raw-data analyser for the NUCLEO-H755ZI-Q capture firmware.

Protocol:
    PC  -> MCU : b'c'                       (request one frame)
    MCU -> PC  : "INFO ...\r\n"             (status, printed here)
                 "FRAME <nbytes>\n"         (header)
                 <nbytes raw RGB565 bytes>  (camera byte order: high byte first)

Modes:
    python ov7670_live.py COM5 --once        grab ONE frame, analyse, plot, save
    python ov7670_live.py COM5               live view (close window to stop)
    python ov7670_live.py --file frame.bin   analyse + plot a saved frame, no board

The analysis finds the real line length from the data itself (the colour-bar
pattern repeats every line), so a wrong camera geometry shows up as numbers.

Install once:
    python -m pip install pyserial numpy pillow matplotlib
Add --swap if colours look wrong (other byte order).
Options: --baud 2000000 (must match the firmware), --rot 1/2/3 (rotate 90/180/270).
"""
import argparse
import sys

import numpy as np

BAUD = 2000000


# ---------------------------------------------------------------- decoding
def rgb565_to_rgb888(raw, w, h, swap=False):
    a = np.frombuffer(raw[: w * h * 2], dtype=np.uint8).reshape(h, w, 2)
    hi, lo = (a[..., 1], a[..., 0]) if swap else (a[..., 0], a[..., 1])
    v = (hi.astype(np.uint16) << 8) | lo.astype(np.uint16)
    r = ((v >> 11) & 0x1F) * 255 // 31
    g = ((v >> 5) & 0x3F) * 255 // 63
    b = (v & 0x1F) * 255 // 31
    return np.dstack((r, g, b)).astype(np.uint8)


def frame_to_rgb(raw, lines, width, swap=False):
    """Decode line by line: bytes/line = len(raw) // lines. An odd line length
    (e.g. 639) gets its last byte dropped, so every line starts on the high
    byte of a pixel and no stripes/slant appear. Returns (rgb, w, bytes/line)."""
    lb = len(raw) // lines
    w = width or lb // 2
    a = np.frombuffer(raw[: lb * lines], dtype=np.uint8).reshape(lines, lb)[:, : w * 2]
    return rgb565_to_rgb888(a.tobytes(), w, lines, swap), w, lb


# ---------------------------------------------------------------- analysis
def analyse(raw):
    """Print byte-repeat statistics and the line period found in the data.
    Returns the best line period in bytes (or None)."""
    a = np.frombuffer(raw, dtype=np.uint8).astype(np.int16)
    n = len(a)
    print("\n=== RAW DATA ANALYSIS ===")
    print(f"total bytes          : {n}  (expected 160x120x2 = 38400 -> ratio {n / 38400:.3f})")
    if n < 8000:
        print("too little data to analyse")
        return None

    # 1) byte repetition: if PCLK is not divided, each byte is sampled ~4 times
    for k in (1, 2, 4):
        eq = np.mean(a[:-k] == a[k:]) * 100
        print(f"byte[i] == byte[i+{k}] : {eq:5.1f} %")
    print("  (colour bars, normal data: +2 high, +1 low.  each byte sampled 4x: +1 high too)")

    # 2) line period: the bar pattern repeats every line -> minimum difference
    seg = a[: min(n, 60000)]
    lags = np.arange(64, min(4000, len(seg) // 3))
    diff = np.array([np.mean(np.abs(seg[:-L] - seg[L:])) for L in lags])
    # multiples of the true period match equally well -> take the SMALLEST lag
    # whose difference is close to the global minimum
    thr = diff.min() + 0.1 * (np.median(diff) - diff.min())
    i0 = int(np.argmax(diff <= thr))
    win = slice(i0, min(i0 + 48, len(lags)))           # refine to the exact minimum
    period = int(lags[win][np.argmin(diff[win])])
    order = np.argsort(diff)
    best = []
    for i in order:                      # distinct candidates, for information
        L = int(lags[i])
        if all(abs(L - b) > 8 for b, _ in best):
            best.append((L, float(diff[i])))
        if len(best) == 5:
            break
    print("good repeat periods  : " + ", ".join(f"{L} B (diff {d:.1f})" for L, d in sorted(best)))
    print(f"-> line length ~{period} bytes = {period / 2:.0f} px of RGB565 "
          f"(expected 320 B = 160 px);  lines in data ~{n / period:.1f}")
    print("=========================\n")
    return period


# ---------------------------------------------------------------- serial
def grab(ser):
    """Request one frame, return raw bytes (or None on failure)."""
    ser.reset_input_buffer()
    ser.write(b"c")
    while True:
        line = ser.readline()
        if not line:
            print("Timeout: no reply. Firmware running? Right COM port?")
            return None
        text = line.decode(errors="replace").strip()
        if text.startswith("FRAME"):
            parts = text.split()
            if len(parts) < 2 or not parts[1].isdigit():
                print("MCU sent 'FRAME' without a byte count: that is the OLD "
                      "boot-time capture code. Remove it from USER CODE 2.")
                continue
            n = int(parts[1])
            break
        if text:
            print("MCU:", text)
    old = ser.timeout
    ser.timeout = n / (BAUD / 10) + 3          # transfer time + margin
    raw = ser.read(n)
    ser.timeout = old
    if len(raw) != n:
        print(f"Short frame: got {len(raw)} of {n} bytes")
        return None
    return raw


# ---------------------------------------------------------------- display
def rotate(rgb, k):
    return np.rot90(rgb, k) if k else rgb


def show_once(raw, swap, lines, width, rot=0):
    import matplotlib.pyplot as plt
    if len(raw) < 1000:
        print("No image data received (0 or almost 0 bytes): nothing to show.")
        return
    analyse(raw)
    rgb, w, lb = frame_to_rgb(raw, lines, width, swap)
    h = lines
    print(f"decoded {lines} lines x {lb} bytes/line -> {w} x {h} px")
    rgb = rotate(rgb, rot)
    from PIL import Image
    hh, ww = rgb.shape[:2]
    Image.fromarray(np.ascontiguousarray(rgb), "RGB").resize((ww * 2, hh * 2), Image.NEAREST).save("frame.png")
    print(f"Saved frame.png ({w} x {h}) and frame.bin")
    plt.imshow(rgb, interpolation="nearest")
    plt.title(f"{w} x {h} px  ({lb} bytes/line)  {len(raw)} bytes")
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="COM port, e.g. COM5")
    ap.add_argument("--swap", action="store_true", help="other byte order")
    ap.add_argument("--once", action="store_true", help="one frame: analyse, plot, save")
    ap.add_argument("--file", help="analyse + plot a saved frame.bin (no board needed)")
    ap.add_argument("--lines", type=int, default=240, help="image height (live mode)")
    ap.add_argument("--width", type=int, default=0, help="force width (px)")
    ap.add_argument("--baud", type=int, default=115200, help="must match firmware")
    ap.add_argument("--rot", type=int, default=0, choices=[0, 1, 2, 3],
                    help="rotate by 90 deg steps (counter-clockwise)")
    args = ap.parse_args()
    global BAUD
    BAUD = args.baud

    if args.file:
        show_once(open(args.file, "rb").read(), args.swap, args.lines, args.width, args.rot)
        return
    if not args.port:
        ap.error("give a COM port, or --file frame.bin")

    import serial
    with serial.Serial(args.port, BAUD, timeout=3) as ser:
        print(f"Opened {args.port} at {BAUD} baud")
        if args.once:
            raw = None
            for _ in range(3):                    # first 'c' may hit the boot phase
                raw = grab(ser)
                if raw is not None:
                    break
            if raw is None:
                sys.exit(1)
            open("frame.bin", "wb").write(raw)
            show_once(raw, args.swap, args.lines, args.width, args.rot)
            return

        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots()
        im = None
        while plt.fignum_exists(fig.number):
            raw = grab(ser)
            if raw is None:
                plt.pause(0.5)
                continue
            if len(raw) // (2 * args.lines) < 16:                               # truncated capture: skip it
                print(f"Skipped short frame ({len(raw)} bytes)")
                continue
            open("frame.bin", "wb").write(raw)
            rgb, w, lb = frame_to_rgb(raw, args.lines, args.width, args.swap)
            rgb = rotate(rgb, args.rot)
            if im is None or im.get_array().shape != rgb.shape:
                ax.clear()
                im = ax.imshow(rgb, interpolation="nearest")
            else:
                im.set_data(rgb)
            ax.set_title(f"{w} x {args.lines}  ({lb} B/line, {len(raw)} B, {BAUD} baud)")
            plt.pause(0.01)


if __name__ == "__main__":
    main()