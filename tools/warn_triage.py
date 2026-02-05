#!/usr/bin/env python3
import argparse
import os
import re
import sys
from collections import Counter, defaultdict

WARN_RE = re.compile(r"^(?P<file>[^:\s][^:]*):(?P<line>\d+):(?P<col>\d+):\s+warning:\s+(?P<msg>.*)$")
KCONFIG_WARN_RE = re.compile(r"^(warning|info):\s+(?P<msg>.*)$")


def bucket_for_path(path: str) -> str:
    norm = path.replace("\\", "/")
    if "/main/" in norm or norm.endswith("/main") or norm.startswith("main/"):
        return "app/main"
    if "/components/" in norm or norm.startswith("components/"):
        return "app/components"
    if "/managed_components/" in norm or norm.startswith("managed_components/"):
        return "managed_components"
    if "/esp-matter/" in norm or norm.startswith("esp-matter/"):
        return "esp-matter"
    if "/connectedhomeip/" in norm:
        return "connectedhomeip"
    return "other"


def parse_log(path: str):
    warnings = []
    kconfig = []
    for line in open(path, "r", errors="replace"):
        line = line.rstrip("\n")
        m = WARN_RE.match(line)
        if m:
            warnings.append(m.groupdict())
            continue
        km = KCONFIG_WARN_RE.match(line)
        if km:
            kconfig.append(km.group("msg"))
    return warnings, kconfig


def summarize(warnings, kconfig):
    by_bucket = Counter()
    by_file = Counter()
    by_msg = Counter()

    for w in warnings:
        bucket = bucket_for_path(w["file"])
        by_bucket[bucket] += 1
        by_file[w["file"]] += 1
        by_msg[w["msg"]] += 1

    print("Warning Summary")
    print(f"Total warnings: {len(warnings)}")
    if kconfig:
        print(f"Kconfig notices: {len(kconfig)}")
    print("")

    if by_bucket:
        print("By Bucket:")
        for bucket, count in by_bucket.most_common():
            print(f"  {bucket}: {count}")
        print("")

    if by_file:
        print("Top Files:")
        for file, count in by_file.most_common(10):
            print(f"  {file}: {count}")
        print("")

    if by_msg:
        print("Top Warning Messages:")
        for msg, count in by_msg.most_common(10):
            print(f"  {count}x {msg}")
        print("")

    if kconfig:
        print("Kconfig Notices:")
        for msg in kconfig[:20]:
            print(f"  {msg}")
        if len(kconfig) > 20:
            print(f"  ... ({len(kconfig) - 20} more)")


def main():
    parser = argparse.ArgumentParser(description="Summarize warnings from ESP-IDF build logs.")
    parser.add_argument("log", help="Path to build log file")
    args = parser.parse_args()

    if not os.path.exists(args.log):
        print(f"Log file not found: {args.log}", file=sys.stderr)
        return 1

    warnings, kconfig = parse_log(args.log)
    summarize(warnings, kconfig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
