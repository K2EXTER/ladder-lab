"""Log the stage A DMM (dmm/firmware/dmm_stageA) to CSV and plot the result.

Usage:
  python dmmlog.py stream COM5 "0 com" --seconds 60 -o ain0.csv
      one reading per line for 60 s (the firmware's "stream" command)
  python dmmlog.py repeat COM5 "refdiff 20" --every 60 --count 120 -o refdiff.csv
      run one command again and again (drift over hours, warming up, ...)
  python dmmlog.py plot ain0.csv [-o ain0.png] [--column V]
      value over time, histogram, mean and standard deviation

  Use "fake" instead of a port name to try the script without the hardware.

Needs: pip install pyserial matplotlib
"""

import argparse
import csv
import math
import random
import sys
import time
from datetime import datetime


# ---------- talking to the board ----------

class FakeDmm:
    """Answers like the firmware, so the script can be tried without hardware."""

    def __init__(self):
        self.out = []
        self.streaming = False
        self.t0 = time.time()

    def write(self, data):
        line = data.decode().strip()
        if self.streaming:
            self.streaming = False
            return
        if line.startswith("stream"):
            self.streaming = True
            self.out.append("stream,ms,code,V   (send any key to stop)")
        elif line.startswith("refdiff"):
            dv = 0.00042 + random.gauss(0, 0.3e-6)
            code = dv * 128 * 8388608 / 2.5
            self.out.append(f"refdiff,AIN4-AIN5,gain=128,ref=int,n=20,code={code:.1f},V={dv:.7f},"
                            f"uV={dv * 1e6:.3f},sd_uV=0.31")
            self.out.append(f"refdiff,ppm_of_2V5,{dv / 2.5 * 1e6:.1f},within both datasheet limits")
        else:
            self.out.append("error,unknown command (type help)")

    def readline(self):
        if self.streaming:
            time.sleep(0.05)
            ms = int((time.time() - self.t0) * 1000)
            v = 1.2500000 + 2e-6 * math.sin(ms / 3000) + random.gauss(0, 1.5e-6)
            return f"s,{ms},{round(v * 8388608 / 2.5)},{v:.7f}\n".encode()
        if self.out:
            return (self.out.pop(0) + "\n").encode()
        time.sleep(0.05)
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


def open_port(port):
    if port == "fake":
        return FakeDmm()
    try:
        import serial
    except ImportError:
        sys.exit("pyserial がありません: pip install pyserial")
    try:
        s = serial.Serial(port, 115200, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"{port} を開けません ({e})。ポート名(デバイスマネージャ / Arduino IDE の「ポート」)と、"
                 "シリアルモニタが開いたままでないかを確かめてください。")
    # Opening the port resets the Mega; wait for the banner to finish.
    time.sleep(2.5)
    s.reset_input_buffer()
    return s


def send(port, command):
    port.write((command + "\n").encode())


def read_line(port):
    raw = port.readline()
    return raw.decode(errors="replace").strip() if raw else ""


# ---------- parsing ----------

def parse(line):
    """'v,AIN0-COM,gain=1,V=1.25' -> {'kind': 'v', 'f1': 'AIN0-COM', 'gain': '1', 'V': '1.25'}"""
    parts = line.split(",")
    row = {"kind": parts[0]}
    for i, p in enumerate(parts[1:], 1):
        if "=" in p:
            k, v = p.split("=", 1)
            row[k.strip()] = v.strip()
        else:
            row[f"f{i}"] = p.strip()
    return row


def write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


# ---------- modes ----------

def cmd_stream(a):
    port = open_port(a.port)
    send(port, "stream " + a.inputs)
    rows, t_end = [], time.time() + a.seconds
    try:
        while time.time() < t_end:
            line = read_line(port)
            if line.startswith("error"):
                sys.exit("ファームが拒否しました: " + line)
            if line.startswith("s,"):
                _, ms, code, v = line.split(",")
                rows.append({"t_s": int(ms) / 1000.0, "code": int(code), "V": float(v)})
                if a.echo:
                    print(line)
    except KeyboardInterrupt:
        pass
    send(port, "x")  # any key stops the stream
    port.close()
    if not rows:
        sys.exit("読み値が1つも来ませんでした。`id` がシリアルモニタで通るか、先に確かめてください。")
    t0 = rows[0]["t_s"]
    for r in rows:
        r["t_s"] = round(r["t_s"] - t0, 3)
    write_csv(a.output, rows)
    print(f"{len(rows)} 行を {a.output} に保存しました")
    summary([r["V"] for r in rows], "V")


def cmd_repeat(a):
    port = open_port(a.port)
    rows, t0 = [], time.time()
    try:
        for i in range(a.count):
            send(port, a.command)
            got, deadline = False, time.time() + a.timeout
            while time.time() < deadline:
                line = read_line(port)
                if not line:
                    if got:
                        break  # the answer is complete
                    continue
                if line.startswith("error"):
                    print("ファームのエラー: " + line)
                    break
                row = parse(line)
                if "V" in row or "value" in row or row["kind"] in ("mon", "noise"):
                    row = {"t_s": round(time.time() - t0, 1), "time": datetime.now().strftime("%H:%M:%S"), **row}
                    rows.append(row)
                    print(line)
                    got = True
            if i + 1 < a.count:
                time.sleep(max(0.0, t0 + (i + 1) * a.every - time.time()))
    except KeyboardInterrupt:
        print("止めました")
    port.close()
    if not rows:
        sys.exit("答えが1つも来ませんでした。コマンドをシリアルモニタで試してください。")
    write_csv(a.output, rows)
    print(f"{len(rows)} 行を {a.output} に保存しました")


def summary(values, unit):
    n = len(values)
    mean = sum(values) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in values) / (n - 1)) if n > 1 else 0.0
    if unit == "V":
        print(f"n={n}  平均={mean:.7f} V  標準偏差={sd * 1e6:.3f} uV  "
              f"最小〜最大={min(values):.7f}〜{max(values):.7f}")
    else:
        print(f"n={n}  平均={mean:.6g} {unit}  標準偏差={sd:.4g} {unit}  最小〜最大={min(values):.6g}〜{max(values):.6g}")
    return mean, sd


def cmd_plot(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(a.csv, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows or a.column not in rows[0]:
        sys.exit(f"{a.csv} に列 {a.column} がありません(列: {', '.join(rows[0]) if rows else 'なし'})")
    rows = [r for r in rows if r.get(a.column) not in (None, "")]
    t = [float(r["t_s"]) for r in rows]
    v = [float(r[a.column]) for r in rows]
    # Values in volts are shown as uV from the mean; other columns (uV, TEMP_C ...) as they are.
    scale = 1e6 if a.column == "V" else 1.0
    unit = "µV" if a.column in ("V", "uV", "sd_uV") else a.column
    mean, sd = summary(v, a.column)
    dev_uv = [(x - mean) * scale for x in v]

    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "sans-serif"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4), gridspec_kw={"width_ratios": [3, 1]})
    ax1.plot(t, dev_uv, lw=0.8)
    ax1.set_xlabel("時間 (s)")
    ax1.set_ylabel(f"平均からのずれ ({unit})  平均 = {mean:.7g} {a.column}")
    ax1.grid(alpha=0.3)
    ax2.hist(dev_uv, bins=min(40, max(5, len(v) // 10)), orientation="horizontal")
    ax2.set_xlabel("回数")
    ax2.set_title(f"σ = {sd * scale:.3g} {unit}  (n = {len(v)})", fontsize=10)
    ax2.sharey(ax1)
    fig.suptitle(a.title or a.csv)
    fig.tight_layout()
    out = a.output or a.csv.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=120)
    print(f"グラフを {out} に保存しました")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    s = sub.add_parser("stream", help="1回ごとの読み値を記録する")
    s.add_argument("port", help="COM5 など(fake で機器なしの試し)")
    s.add_argument("inputs", help='"0 com" や "4 5 128"(ファームの stream の引数)')
    s.add_argument("--seconds", type=float, default=30)
    s.add_argument("-o", "--output", default="stream.csv")
    s.add_argument("--echo", action="store_true", help="受け取った行も表示する")
    s.set_defaults(func=cmd_stream)

    r = sub.add_parser("repeat", help="1つのコマンドを繰り返して記録する")
    r.add_argument("port")
    r.add_argument("command", help='"refdiff 20" や "v 0 com 1 10" や "mon"')
    r.add_argument("--every", type=float, default=10, help="何秒ごとに(既定 10)")
    r.add_argument("--count", type=int, default=10)
    r.add_argument("--timeout", type=float, default=30, help="1回の答えを待つ秒数")
    r.add_argument("-o", "--output", default="repeat.csv")
    r.set_defaults(func=cmd_repeat)

    g = sub.add_parser("plot", help="CSV をグラフにする")
    g.add_argument("csv")
    g.add_argument("-o", "--output")
    g.add_argument("--column", default="V")
    g.add_argument("--title")
    g.set_defaults(func=cmd_plot)

    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
