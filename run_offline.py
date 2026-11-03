"""Run selected strategies against synthetic local cases only."""
from pathlib import Path
import argparse
import json
import socket
import sys
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "algorithms"))
sys.path.insert(0, str(ROOT / "algorithms" / "vendor"))
def denied(*args, **kwargs):
    raise RuntimeError("This showcase entry runs offline only")
socket.create_connection = denied
socket.getaddrinfo = denied
socket.socket.connect = denied
socket.socket.connect_ex = denied
from experiment_runner import run_one
parser = argparse.ArgumentParser()
parser.add_argument("--question", type=int, choices=(3, 4), default=3)
parser.add_argument("--seed", type=int, default=20260912)
parser.add_argument("--output-dir", type=Path, default=ROOT / "runs")
args = parser.parse_args()
candidate = "Q3_X13" if args.question == 3 else "Q4_X41"
summary = run_one(args.question, candidate, args.seed, args.output_dir, offline=True)
print(json.dumps({k: summary.get(k) for k in ("candidate", "completed", "clear_success", "truth_total", "virtual_time_s", "failure")}, ensure_ascii=False))
raise SystemExit(0 if summary.get("completed") and not summary.get("failure") else 1)
