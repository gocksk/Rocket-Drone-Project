"""로그 → CSV. 엑셀·판다스로 볼 때 쓴다.

    python -m doe.export runs/smoke.jsonl              → runs/smoke.csv
    python -m doe.export runs/smoke.jsonl --only dv,ec  설계변수와 성적표만

**원본은 JSONL 이고 CSV 는 사본이다.** 되돌려 읽지 않는다 — 분석(`doe.report`)도
재개(`doe.run --resume`)도 JSONL 을 본다. 진단 항목이 계속 느는 중이라 열이 고정된
형식을 원본으로 삼으면 과거 로그와 안 붙기 때문이다.

열 이름은 묶음을 점으로 잇는다:  `dv.d_body` · `g.g5` · `ec.C1_MTOW[kg]` · `diag.kv`
열 순서는 행에 나온 순서대로 고정하고, 어떤 행에 없는 열은 빈 칸으로 둔다
(탈락 행은 g·ec 가 통째로 비어 있다 — 그게 정보다).

숫자는 파이썬 repr 그대로 적는다. 반올림하면 순수성 비교(같은 설계점이 같은 값을
냈는지)가 CSV 에서 깨진다.
"""
from __future__ import annotations

import csv
import json
import os
import sys

from common.out import stdout_utf8

SECTIONS = ("dv", "g", "ec", "diag")
TOP = ("run", "id", "dup_of", "feasible", "fail_code", "fail_stage", "wall_s")


def flatten(row: dict, only=SECTIONS) -> dict:
    """중첩 묶음을 `묶음.키` 열로 편다. dict·list 값은 JSON 문자열로 남긴다."""
    out = {n: row.get(n) for n in TOP}
    for sec in only:
        for k, v in (row.get(sec) or {}).items():
            out[f"{sec}.{k}"] = (json.dumps(v, ensure_ascii=False)
                                 if isinstance(v, (dict, list)) else v)
    return out


def convert(log_path: str, csv_path: str, only=SECTIONS) -> tuple:
    rows = []
    with open(log_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    print("⚠ 깨진 줄 하나를 건너뛴다")
    # 로그는 완료 순서라 여기서 정렬한다. 실행 도장을 앞에 두는 이유는 한 파일에
    # 여러 실행이 섞였을 때 id 가 겹치기 때문이다 — id 는 실행 안에서만 유일하다.
    rows.sort(key=lambda r: (r.get("run") or "", r.get("id", 0)))
    ids = {}
    for r in rows:
        ids[(r.get("run"), r.get("id"))] = ids.get((r.get("run"), r.get("id")), 0) + 1
    dup = sum(1 for c in ids.values() if c > 1)
    if dup:
        print(f"⚠ 같은 (실행, id) 가 {dup} 쌍 겹친다 — 한 파일에 여러 실행이 섞였다.")
    flat = [flatten(r, only) for r in rows]

    cols = []
    for r in flat:                              # 처음 나온 순서대로 열을 쌓는다
        for k in r:
            if k not in cols:
                cols.append(k)

    # utf-8-sig — BOM 이 없으면 엑셀이 한글·µ 를 깨서 연다 (Windows 기본 인코딩 탓)
    with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(flat)
    return len(flat), len(cols)


def main_cli(argv=None) -> int:
    stdout_utf8()
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print("사용법: python -m doe.export <로그.jsonl> [--out 파일.csv] "
              "[--only dv,g,ec,diag]")
        return 2
    log = argv[0]
    out = None
    only = SECTIONS
    for i, a in enumerate(argv):
        if a == "--out" and i + 1 < len(argv):
            out = argv[i + 1]
        if a == "--only" and i + 1 < len(argv):
            only = tuple(x.strip() for x in argv[i + 1].split(",") if x.strip())
    bad = [s for s in only if s not in SECTIONS]
    if bad:
        print(f"--only 에 모르는 묶음이 있다: {bad}  (가능: {', '.join(SECTIONS)})")
        return 2
    if not os.path.exists(log):
        print(f"{log} 이 없다.")
        return 2
    out = out or (log[:-6] if log.endswith(".jsonl") else log) + ".csv"
    n, c = convert(log, out, only)
    print(f"{n} 행 × {c} 열 → {out}")
    print("  원본은 JSONL 이다. 분석·재개는 그쪽을 본다 — CSV 는 사본이다.")
    return 0


if __name__ == "__main__":
    sys.exit(main_cli())
