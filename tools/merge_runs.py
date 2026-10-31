"""把一次（或多次）采集的 trace_*.npz 合并成一个文件。

用法（在仓库根目录）：
    python tools/merge_runs.py remote_runs\\20261022_101500_0001
    python tools/merge_runs.py remote_runs          # 合并下面所有采集文件夹
也可以把文件夹直接拖到【合并采集数据】.bat 上。

规则：原始单道文件全部保留；结果写到 <采集文件夹>/<采集名>_merged.npz；
校准采集（结尾 _c）、空闲采集（结尾 _idle）和 _ 开头的系统文件夹默认跳过。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from run_archive import RunArchiveError, list_trace_files, merge_run  # noqa: E402


def _is_skipped_name(name: str) -> bool:
    return name.startswith("_") or name.endswith("_c") or name.endswith("_idle")


def find_run_dirs(path: Path, include_all: bool = False) -> list[Path]:
    path = Path(path)
    if not path.is_dir():
        return []
    if list_trace_files(path):
        return [path]
    result = []
    for child in sorted(path.iterdir()):
        if not child.is_dir():
            continue
        if not include_all and _is_skipped_name(child.name):
            continue
        if list_trace_files(child):
            result.append(child)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="合并采集数据（trace_*.npz → 一个文件）")
    parser.add_argument("paths", nargs="+", type=Path, help="采集文件夹，或包含多个采集文件夹的上级目录")
    parser.add_argument("--all", action="store_true", help="连校准(_c)/空闲(_idle)采集也合并")
    args = parser.parse_args(argv)

    run_dirs: list[Path] = []
    for path in args.paths:
        found = find_run_dirs(path, include_all=args.all)
        if not found:
            print(f"[跳过] 没找到采集数据：{path}")
        run_dirs.extend(found)

    failures = 0
    for run_dir in run_dirs:
        try:
            report = merge_run(run_dir)
        except RunArchiveError as exc:
            failures += 1
            print(f"[失败] {run_dir.name}：{exc}")
            continue
        print(f"[完成] {report.summary_text()}")
        for item in report.skipped_files[:20]:
            print(f"        跳过 {item.get('file')}：{item.get('reason')}")
    print(f"共处理 {len(run_dirs)} 个采集，失败 {failures} 个。")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
