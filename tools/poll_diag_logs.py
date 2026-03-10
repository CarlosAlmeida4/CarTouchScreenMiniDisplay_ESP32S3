#!/usr/bin/env python3
"""Poll diagnostics logs from the ESP32 device and store them locally.

Examples:
  python tools/poll_diag_logs.py --host 192.168.1.90
  python tools/poll_diag_logs.py --host 192.168.1.90 --output logs.txt --cursor-file .diag.cursor
  python tools/poll_diag_logs.py --host 192.168.1.90 --token my-secret-token --status-every 15
"""

from __future__ import annotations

import argparse
import json
import pathlib
import signal
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Poll ESP diagnostics logs via /diag/logs")
    parser.add_argument("--host", required=True, help="Device hostname or IP (for example: 192.168.1.90)")
    parser.add_argument("--port", type=int, default=80, help="HTTP port on the device (default: 80)")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds (default: 1.0)")
    parser.add_argument("--limit", type=int, default=80, help="Max lines fetched per request (default: 80, max: 200)")
    parser.add_argument("--timeout", type=float, default=3.0, help="HTTP timeout in seconds (default: 3.0)")
    parser.add_argument("--token", default=None, help="Optional diagnostics token sent as X-Diag-Token")
    parser.add_argument(
        "--cursor-file",
        default=".diag_cursor",
        help="Path to persist next cursor between runs (default: .diag_cursor)",
    )
    parser.add_argument("--start-cursor", type=int, default=None, help="Override cursor-file with explicit start cursor")
    parser.add_argument("--output", default=None, help="Output file path. If omitted, logs are printed to stdout")
    parser.add_argument(
        "--raw-json",
        action="store_true",
        help="Write each log entry as JSON line instead of formatted text",
    )
    parser.add_argument(
        "--status-every",
        type=int,
        default=30,
        help="Print poll status every N successful requests (default: 30)",
    )
    parser.add_argument("--once", action="store_true", help="Fetch once and exit")
    return parser.parse_args()


def build_url(host: str, port: int, cursor: int, limit: int) -> str:
    query = urllib.parse.urlencode({"cursor": cursor, "limit": limit})
    return f"http://{host}:{port}/diag/logs?{query}"


def read_cursor(path: pathlib.Path, fallback: int = 0) -> int:
    if not path.exists():
        return fallback
    try:
        raw = path.read_text(encoding="utf-8").strip()
        return int(raw) if raw else fallback
    except (OSError, ValueError):
        return fallback


def write_cursor(path: pathlib.Path, cursor: int) -> None:
    path.write_text(str(cursor), encoding="utf-8")


def fetch_logs(url: str, token: str | None, timeout: float) -> dict[str, Any]:
    req = urllib.request.Request(url, method="GET")
    req.add_header("Accept", "application/json")
    if token:
        req.add_header("X-Diag-Token", token)

    with urllib.request.urlopen(req, timeout=timeout) as response:
        body = response.read()

    return json.loads(body.decode("utf-8"))


def format_line(entry: dict[str, Any]) -> str:
    seq = entry.get("seq", "?")
    ts_us = entry.get("ts_us", 0)
    msg = entry.get("msg", "")
    ts_s = float(ts_us) / 1_000_000.0
    return f"[{seq}] {ts_s:12.6f}s {msg}"


def emit_lines(entries: list[dict[str, Any]], output_path: pathlib.Path | None, raw_json: bool) -> None:
    if output_path is None:
        for entry in entries:
            if raw_json:
                print(json.dumps(entry, ensure_ascii=True), flush=True)
            else:
                print(format_line(entry), flush=True)
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("a", encoding="utf-8") as fp:
        for entry in entries:
            if raw_json:
                fp.write(json.dumps(entry, ensure_ascii=True) + "\n")
            else:
                fp.write(format_line(entry) + "\n")


def main() -> int:
    args = parse_args()
    limit = max(1, min(200, args.limit))

    cursor_path = pathlib.Path(args.cursor_file)
    cursor = args.start_cursor if args.start_cursor is not None else read_cursor(cursor_path, 0)

    output_path = pathlib.Path(args.output) if args.output else None

    should_stop = False

    def _handle_signal(signum: int, frame: Any) -> None:
        del signum, frame
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, _handle_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _handle_signal)

    success_count = 0
    print(
        f"Polling http://{args.host}:{args.port}/diag/logs every {args.interval}s (cursor={cursor}, limit={limit})",
        file=sys.stderr,
        flush=True,
    )

    while not should_stop:
        url = build_url(args.host, args.port, cursor, limit)
        try:
            payload = fetch_logs(url, args.token, args.timeout)
            logs = payload.get("logs", [])
            next_cursor = int(payload.get("next_cursor", cursor))
            dropped = int(payload.get("dropped", 0))

            if isinstance(logs, list) and logs:
                emit_lines(logs, output_path, args.raw_json)

            if next_cursor != cursor:
                cursor = next_cursor
                write_cursor(cursor_path, cursor)

            success_count += 1
            if args.status_every > 0 and success_count % args.status_every == 0:
                print(
                    f"status: cursor={cursor}, fetched={len(logs)}, dropped={dropped}",
                    file=sys.stderr,
                    flush=True,
                )

            if args.once:
                break

            time.sleep(max(0.05, args.interval))

        except urllib.error.HTTPError as exc:
            print(f"HTTP error: {exc.code} {exc.reason}", file=sys.stderr, flush=True)
            if exc.code == 401:
                print("Unauthorized. Check --token or device auth config.", file=sys.stderr, flush=True)
                return 2
            time.sleep(max(0.25, args.interval))
        except urllib.error.URLError as exc:
            print(f"Network error: {exc.reason}", file=sys.stderr, flush=True)
            time.sleep(max(0.25, args.interval))
        except json.JSONDecodeError as exc:
            print(f"Invalid JSON response: {exc}", file=sys.stderr, flush=True)
            time.sleep(max(0.25, args.interval))
        except Exception as exc:  # noqa: BLE001 - keep tool alive for long-running collection
            print(f"Unexpected error: {exc}", file=sys.stderr, flush=True)
            time.sleep(max(0.25, args.interval))

    write_cursor(cursor_path, cursor)
    print(f"Stopped. Final cursor={cursor}", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
