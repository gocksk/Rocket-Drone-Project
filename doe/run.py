"""DOE 배치 런처 — 표본을 뿌리고, 병렬로 돌리고, 한 줄씩 로그에 적는다.

    python -m doe.run --box smoke --n 30                    배관 확인
    python -m doe.run --box smoke --n 30 --dry-run          상자만 보고 끝
    python -m doe.run --box screen --n 500 --set d_body=0.07:0.11 ...
    python -m doe.run --box smoke --n 30 --resume           이어 돌리기

**여기는 조립만 한다.** 물리는 `main.evaluate` 가 하고, 이 파일은 그걸 부르는
순서와 로그·재개·매니페스트만 책임진다. 사이징 경로는 한 줄도 안 바꾼다.

산출물 두 개:
  <out>.jsonl           설계점당 한 줄. 완료 순서대로 붙는다 (정렬은 분석에서)
  <out>.manifest.json   상자·seed·커밋·상수 해시. **로그만 있고 이게 없으면 못 믿는다** —
                        상수가 TBD 투성이라 며칠 뒤 두 로그가 다른 상수에서 나온 것을
                        모르면 결론이 통째로 오염된다.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import dataclasses
import functools
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime

from common.out import stdout_utf8
from doe import row, sample, space, worker

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git() -> dict:
    """어느 코드에서 나온 로그인지. git 이 없으면 없다고 적는다 — 지어내지 않는다."""
    def _run(*a):
        return subprocess.run(a, cwd=_ROOT, capture_output=True, text=True,
                              timeout=10).stdout.strip()
    try:
        return {"commit": _run("git", "rev-parse", "--short", "HEAD"),
                "dirty": bool(_run("git", "status", "--porcelain"))}
    except Exception:
        return {"commit": None, "dirty": None}


def _sha(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def _fmt_t(s: float) -> str:
    m, s = divmod(int(s), 60)
    h, m = divmod(m, 60)
    return f"{h:d}:{m:02d}:{s:02d}"


def _load_ids(path: str) -> set:
    """재개용 — 이미 적힌 id. 깨진 줄은 건너뛴다 (중간에 죽으면 마지막 줄이 잘린다)."""
    ids = set()
    if not os.path.exists(path):
        return ids
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                ids.add(json.loads(line)["id"])
            except Exception:
                continue
    return ids


def parse_args(argv=None):
    p = argparse.ArgumentParser("doe.run", description="DOE 배치 런처")
    p.add_argument("--box", default="smoke", choices=sorted(space.BOXES),
                   help="상자 이름 (기본 smoke = 공칭 ±4%%, 배관 확인용)")
    p.add_argument("--set", dest="sets", action="append", default=[],
                   metavar="VAR=LO:HI", help="축 범위 덮어쓰기 (여러 번 가능)")
    p.add_argument("--focus", dest="focuses", action="append", default=[],
                   metavar="VAR=F", help="공칭점 중심으로 폭을 F 배로 좁힌다")
    p.add_argument("--n", type=int, default=30, help="셀 수 수준당 표본 수")
    p.add_argument("--n-ser", default="6", help="셀 수 수준 (예: 4,6,8)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--dup-frac", type=float, default=0.01,
                   help="순수성 감시용 복제 비율 (0 이면 끔)")
    p.add_argument("--split", action="store_true",
                   help="Ŝ 분해까지 낸다 — 설계점당 +16%%")
    p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    p.add_argument("--out", default=None, help="출력 경로 앞자리 (기본 runs/<box>)")
    p.add_argument("--resume", action="store_true", help="이미 적힌 id 는 건너뛴다")
    p.add_argument("--force", action="store_true",
                   help="매니페스트가 안 맞아도 이어쓴다 (상자가 섞인다)")
    p.add_argument("--dry-run", action="store_true", help="상자와 표본만 찍고 끝")
    return p.parse_args(argv)


def main_cli(argv=None) -> int:
    stdout_utf8()
    a = parse_args(argv)
    out = a.out or os.path.join("runs", a.box)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    log_path, man_path = out + ".jsonl", out + ".manifest.json"

    # ── 상자 ──
    box = space.resolve(a.box, [space.parse_set(s) for s in a.sets],
                        [space.parse_focus(s) for s in a.focuses])
    if space.missing(box):
        print("상자가 비어 있는 축이 있다 — 그대로는 표본을 못 뿌린다.\n")
        for n in space.missing(box):
            print(f"  {n:14s} 미정  (ICD §8 B-2)")
        print("\n  --set 으로 채운다.  예: --set d_body=0.07:0.11")
        print(f"  참고: pd_prop 의 §2 하한 규칙 값은 {space.pd_prop_lo():.4f} 다.")
        return 2
    for w in space.validate(box):
        print(f"⚠ {w}")

    levels = tuple(int(x) for x in a.n_ser.split(",") if x.strip())
    pts = sample.build(box, a.n, a.seed, levels, a.dup_frac)

    print("=" * 74)
    print(f"상자 {a.box}   표본 {len(pts)} 점 "
          f"(수준당 {a.n} × {len(levels)} 수준 + 복제 {a.dup_frac:.0%})")
    print("=" * 74)
    for n in space.AXES:
        lo, hi = box[n]
        print(f"  {n:14s} {lo:12.5f} – {hi:<12.5f}")
    print(f"  {'n_ser':14s} {levels}")

    if a.dry_run:
        print("-" * 74)
        for p in pts[:3]:
            print(f"  #{p.id}  {dataclasses.asdict(p.dv)}")
        print(f"  … 총 {len(pts)} 점.  --dry-run 이라 평가는 하지 않았다.")
        return 0

    # ── 매니페스트 ── 상자가 다른 로그를 한 파일에 섞지 않는다
    man = {"created": datetime.now().isoformat(timespec="seconds"),
           "finished": None, "argv": sys.argv,
           "git": _git(),
           "constants_sha256": _sha(os.path.join(_ROOT, "constants.py")),
           "python": sys.version.split()[0], "platform": platform.platform(),
           "box_name": a.box, "box": space.describe(box), "n_ser": list(levels),
           "n": a.n, "seed": a.seed, "dup_frac": a.dup_frac, "split": a.split,
           "n_points": len(pts), "workers": a.workers,
           "nominal": dataclasses.asdict(space.NOMINAL)}
    key = ("box", "n_ser", "n", "seed", "dup_frac", "split")
    if os.path.exists(man_path):
        with open(man_path, encoding="utf-8") as f:
            old = json.load(f)
        diff = [n for n in key if old.get(n) != man.get(n)]
        if diff and not a.force:
            print(f"\n기존 매니페스트와 다르다: {', '.join(diff)}")
            print(f"  {man_path} 을 지우거나 --out 을 바꾸거나 --force 를 준다.")
            return 2
        if old.get("constants_sha256") != man["constants_sha256"]:
            print("⚠ constants.py 가 이전 실행 이후 바뀌었다 — 두 로그는 다른 상수에서 나온다")
    with open(man_path, "w", encoding="utf-8") as f:
        json.dump(man, f, ensure_ascii=False, indent=1)

    done = _load_ids(log_path) if a.resume else set()
    if done:
        print(f"  재개 — 이미 적힌 {len(done)} 점은 건너뛴다")
    elif os.path.exists(log_path) and os.path.getsize(log_path) > 0 and not a.force:
        print(f"\n{log_path} 에 이미 내용이 있다. --resume 이나 --force 를 준다.")
        return 2
    todo = [p for p in pts if p.id not in done]

    # ── 배치 ──
    work = functools.partial(worker.evaluate_point, split=a.split)
    t0 = time.perf_counter()
    n_done, codes, rows = 0, Counter(), {}
    tick = max(1, len(todo) // 20)
    print("-" * 74)
    with open(log_path, "a", encoding="utf-8") as f:
        with cf.ProcessPoolExecutor(max_workers=a.workers) as ex:
            futs = [ex.submit(work, p) for p in todo]
            try:
                for fut in cf.as_completed(futs):
                    r = fut.result()
                    # 한 줄씩 흘려 쓰고 즉시 flush — 중간에 죽어도 여기까지는 남는다
                    f.write(json.dumps(r, ensure_ascii=False) + "\n")
                    f.flush()
                    rows[r["id"]] = r
                    codes[r["fail_code"] or "OK"] += 1
                    n_done += 1
                    if n_done % tick == 0 or n_done == len(todo):
                        el = time.perf_counter() - t0
                        eta = el / n_done * (len(todo) - n_done)
                        print(f"  {n_done:6d}/{len(todo)}  경과 {_fmt_t(el)}  "
                              f"남음 {_fmt_t(eta)}  ({el / n_done:.3f} s/점)")
            except KeyboardInterrupt:
                print("\n중단 — 여기까지는 로그에 남았다. --resume 으로 이어 돌린다.")
                ex.shutdown(wait=False, cancel_futures=True)
                return 130

    el = time.perf_counter() - t0
    man["finished"] = datetime.now().isoformat(timespec="seconds")
    man["elapsed_s"] = el
    man["counts"] = dict(codes)
    with open(man_path, "w", encoding="utf-8") as f:
        json.dump(man, f, ensure_ascii=False, indent=1)

    # ── 요약 ── (자세한 분석은 doe.report 가 로그를 다시 읽어서 한다)
    ok = sum(1 for r in rows.values() if r["feasible"])
    print("-" * 74)
    print(f"  평가 {n_done} 점   소요 {_fmt_t(el)}   "
          f"실효 {el / max(n_done, 1):.3f} s/점 ({a.workers} 워커)")
    print(f"  합격 {ok} / {n_done}  ({ok / max(n_done, 1):.1%})")
    for code, c in codes.most_common():
        print(f"    {code:28s} {c:6d}  {c / max(n_done, 1):6.1%}")
    print(_purity(rows))
    print("-" * 74)
    print(f"  로그       {log_path}")
    print(f"  매니페스트 {man_path}")
    print(f"  분석       python -m doe.report {log_path}")
    return 0


def _purity(rows: dict) -> str:
    """복제 쌍이 글자까지 같은지 — 병렬에서 설계점끼리 오염되면 여기서 갈라진다."""
    pairs = [(r, rows.get(r["dup_of"])) for r in rows.values()
             if r.get("dup_of") is not None]
    pairs = [(x, y) for x, y in pairs if y is not None]
    if not pairs:
        return "  순수성     복제 쌍 없음 (--dup-frac 0 이거나 원본이 재개로 건너뛰어짐)"
    bad = [x["id"] for x, y in pairs if row.signature(x) != row.signature(y)]
    if bad:
        return (f"  순수성     FAIL — {len(bad)}/{len(pairs)} 쌍이 갈라졌다 "
                f"(id {bad[:5]}). resp_of 순수성이 깨졌다는 뜻이다")
    return f"  순수성     OK — 복제 {len(pairs)} 쌍이 전부 비트 동일"


if __name__ == "__main__":
    sys.exit(main_cli())
