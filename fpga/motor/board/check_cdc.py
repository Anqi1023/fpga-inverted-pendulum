"""Audit this Gowin netlist/report; structural checks, NOT a generic CDC/MTBF tool."""
from pathlib import Path
from html.parser import HTMLParser
import re

ROOT = Path(__file__).resolve().parent

def require(ok, message):
    if not ok:
        raise SystemExit("CDC_REVIEW_FAIL: " + message)

class Text(HTMLParser):
    def __init__(self):
        super().__init__(); self.rows = []
    def handle_data(self, data):
        if data.strip(): self.rows.append(data.strip())

vg = ROOT / "impl/gwsynthesis/dock_motor_test.vg"
report = ROOT / "impl/pnr/dock_motor_test_tr_content.html"
require(vg.exists() and report.exists(), "Run gw_sh build.tcl first")
for source in [*ROOT.glob("*.sdc"), *ROOT.glob("*.cst"), *ROOT.glob("*.gprj"),
               ROOT / "build.tcl", *(ROOT.parent / "rtl").glob("*.v")]:
    require(report.stat().st_mtime >= source.stat().st_mtime, "Stale build: " + source.name)
modules = {}
for name, body in re.findall(r"module\s+(\w+)\s*\((.*?)endmodule", vg.read_text(), re.S):
    cells = {}
    for kind, inst, pins in re.findall(r"^  (\w+)\s+(\w+)\s*\((.*?)\);", body, re.M | re.S):
        cells[inst] = (kind, dict(re.findall(r"\.(\w+)\(([^()]*)\)", pins)))
    modules[name] = cells

def users(cells, net):
    return {(name, pin) for name, (_, pins) in cells.items()
            for pin, value in pins.items() if value.strip() == net and pin not in {"Q", "O"}}

def pair(cells, first, second):
    require(first in cells and second in cells, "Missing synchronizer register")
    a, b = cells[first], cells[second]
    require(a[0].startswith("DFF") and b[0].startswith("DFF"), "Synchronizer not flip-flops")
    require(a[1]["Q"] == b[1]["D"], "Logic inserted between stages")
    require(users(cells, a[1]["Q"]) == {(second, "D")}, "First stage has extra fanout")

buttons = [c for n,c in modules.items() if n == "button_input" or n.startswith("button_input_")]
require(len(buttons) == 3, "Expected three specialized button modules")
for cells in buttons:
    pair(cells, "sync_meta_s0", "sync_value_s0")
    inv = [pins for kind,pins in cells.values() if kind == "INV" and pins["O"] == cells["sync_meta_s0"][1]["D"]]
    require(len(inv) == 1, "Expected button polarity inverter")
    require(len(users(cells, inv[0]["I"])) == 1, "Raw button has extra fanout")
    require(users(cells, inv[0]["O"]) == {("sync_meta_s0", "D")}, "Raw input bypasses first stage")
r = modules["reset_guard"]
pair(r, "release_sync_0_s0", "release_sync_1_s0")
raw_clear = r["release_sync_0_s0"][1]["CLEAR"]
require(users(r, raw_clear) == {("release_sync_0_s0", "CLEAR"), ("release_sync_1_s0", "CLEAR")}, "Raw reset clears downstream logic")
sync_clear = [pins["O"] for kind,pins in r.values() if kind == "INV" and pins["I"] == r["release_sync_1_s0"][1]["Q"]]
require(len(sync_clear) == 1, "Missing synchronized reset distribution")
for name,(kind,pins) in r.items():
    if kind.startswith("DFF") and not name.startswith("release_sync_"):
        require(pins.get("CLEAR") == sync_clear[0], "Wrong downstream reset: " + name)
print("STRUCTURE_PASS: four pairs, first-stage fanout=1; raw reset clears only synchronizer")

parser = Text(); parser.feed(report.read_text()); text = "\n".join(parser.rows)
for label in ["Numbers of Setup Violated Endpoints", "Numbers of Hold Violated Endpoints"]:
    require(label + "\n0\n" in text, label)
constraints = text.split("Timing Constraints Report:")[-1]
sdc = (ROOT / "dock_motor_test.sdc").read_text()
commands = [line.strip() for line in sdc.splitlines() if line.startswith(("create_clock", "set_false_path", "set_max_delay"))]
require(len(commands) == 10, "Review expected constraint inventory")
for command in commands:
    require("Actived\n" + command in constraints, "Inactive constraint: " + command)
exceptions = text.split("Timing Exceptions Report:")[1].split("Hold Analysis Report")[0]
slacks = re.findall(r"Path Summary:\nSlack\n(-?[\d.]+)", exceptions)
require(len(slacks) == 4 and all(float(s) >= 0 for s in slacks), "Missing/failing interstage max-delay paths")
print("INTERSTAGE_PASS: 5 ns budget, slack ns = " + ", ".join(slacks))
for heading,next_heading in [("Recovery Paths Table", "Removal Paths Table"), ("Removal Paths Table", "Minimum Pulse Width Table")]:
    table = text.split(heading,1)[1].split(next_heading,1)[0]
    paths = re.findall(r"\n\d+\n(-?[\d.]+)\nreset_i/release_sync_1_s0/Q\nreset_i/[^\n]+/CLEAR",table)
    require(len(paths) == 20 and all(float(s) >= 0 for s in paths), "Missing/failing " + heading)
    print(heading + ": 20 endpoints; minimum slack ns = " + str(min(map(float,paths))))
print("HENGXIN_CDC_RESET_REVIEW_PASS")
print("Scope: generated netlist and constrained paths only; no analog MTBF or external electrical signoff.")
