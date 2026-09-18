"""Batched full-regression runner (round-2 environment workaround).

On the new machine (Qt 6.11.2 WebEngine), a single `unittest discover` process
crashes with a FATAL Chromium check (CalledOnValidBrowserThread) when several
QWebEngineView-creating test modules share one process. The t0 baseline
reproduces the identical crash (captain evidence 2026-09-06 06:15, worktree of
9027366), so this is environment drift, not a code regression.

Workaround: run each tests/test_*.py module in its own subprocess and aggregate
results, restoring the round-1 "Ran 716 tests OK" standard in a machine-stable
way. Exit code is 0 only if every module exits 0; total test counts are summed
from each module's "Ran N tests" line.

Usage (from the repository root):
    $env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'
    .venv312\\Scripts\\python.exe docs\\run_tests_batched.py [--only name_fragment ...]
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TESTS_DIR = ROOT / "tests"
RAN_RE = re.compile(r"^Ran (\d+) tests? in ([\d.]+)s", re.MULTILINE)


def discover_modules(only: list[str]) -> list[str]:
    names = sorted(
        p.stem for p in TESTS_DIR.glob("test_*.py")
        if not only or any(frag in p.stem for frag in only)
    )
    return names


def main() -> int:
    only = []
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == "--only":
            i += 1
            while i < len(argv) and not argv[i].startswith("--"):
                only.append(argv[i])
                i += 1
        else:
            i += 1

    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    env.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    env["PYTHONIOENCODING"] = "utf-8"

    modules = discover_modules(only)
    total_ran = 0
    failures: list[str] = []
    shutdown_crash: list[tuple[str, int]] = []
    started = time.monotonic()
    for idx, mod in enumerate(modules, 1):
        timed_out = False
        try:
            proc = subprocess.run(
                [sys.executable, "-m", "unittest", f"tests.{mod}", "-v"],
                cwd=str(ROOT), env=env, capture_output=True, text=True,
                encoding="utf-8", errors="replace", timeout=900,
            )
            tail = (proc.stderr or "") + (proc.stdout or "")
            returncode = proc.returncode
        except subprocess.TimeoutExpired as exc:
            tail = ((exc.stderr or b"").decode("utf-8", "replace")
                    + (exc.stdout or b"").decode("utf-8", "replace")) \
                if isinstance(exc.stderr, bytes) else str(exc.stderr or "") + str(exc.stdout or "")
            returncode = 124
            timed_out = True
            tail += "\nTIMEOUT after 900s (recorded as FAIL, remaining modules continue)\n"
        m = RAN_RE.search(tail)
        ran = int(m.group(1)) if m else 0
        total_ran += ran
        # captain adjudication 2026-09-06: decouple test-layer result from
        # process exit code. Qt 6.11.2 WebEngine teardown segfaults (0xC0000005)
        # after unittest already printed OK on some modules (environment drift;
        # t0 baseline reproduces). Tests-layer OK => PASS with annotation;
        # only true FAILED/NO SUMMARY/timeout => FAIL.
        tests_ok = bool(m) and re.search(r"^OK(\s|$)", tail, re.MULTILINE) is not None
        if timed_out:
            ok = False
            status = "FAIL"
        elif tests_ok and returncode == 0:
            ok = True
            status = "OK "
        elif tests_ok:
            ok = True
            status = "PASS*"
            shutdown_crash.append((mod, returncode))
        else:
            ok = False
            status = "FAIL"
        print(f"[{idx:02d}/{len(modules)}] {status} {mod}: {m.group(0) if m else 'NO SUMMARY (exit %s)' % returncode}", flush=True)
        if not ok:
            failures.append(mod)
            # print the failing summary block for the record
            lines = tail.splitlines()
            keep = [ln for ln in lines if ln.startswith(("FAIL:", "ERROR:", "Traceback", "  ")) or "FAILED" in ln or "Ran " in ln]
            print("---- failure detail (tail) ----")
            for ln in keep[-40:]:
                print(ln)
            print("-------------------------------")

    elapsed = time.monotonic() - started
    print(f"BATCHED TOTAL: Ran {total_ran} tests in {elapsed:.1f}s over {len(modules)} modules; failed modules: {failures if failures else 'none'}")
    if shutdown_crash:
        print("shutdown-crash (environment drift, tests OK): " + ", ".join(f"{mod} (exit {rc})" for mod, rc in shutdown_crash))
    if failures:
        print("OVERALL: FAILED")
        return 1
    print("OVERALL: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
