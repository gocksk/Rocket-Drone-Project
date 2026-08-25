"""DOE 로그 분석 — 로그 파일만 먹는다. 배치를 다시 돌리지 않는다.

    python -m doe.report runs/smoke.jsonl

**실행과 분석을 가르는 이유**: 1 만 점이 수십 분이다. 보는 기준을 바꿀 때마다
배치를 다시 돌리면 안 된다. 로그가 진실의 출처이고 여기는 로그만 읽는다.

읽는 규칙 하나 — **사이징 게이트(g1~g4)와 품질 게이트(g5~g9)를 섞어 읽지 않는다.**
g2·g3·g4 는 사이징이 0 으로 몰아붙이는 잔차라 "가장 빡빡한 게이트"처럼 보이지만
설계점의 성질이 아니다 (docs §11-44). 실제 스크리닝은 g5~g9 에서 일어난다.
"""
from __future__ import annotations

import json
import os
import statistics as st
import sys
from collections import Counter, defaultdict

from common.out import stdout_utf8
from doe import row, space

SIZING_GATES = ("g1", "g2", "g3", "g4")
QUALITY_GATES = ("g5", "g6", "g7", "g8", "g9")


def load(path: str) -> list:
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                print("⚠ 깨진 줄 하나를 건너뛴다 (중단된 배치의 마지막 줄일 수 있다)")
    return rows


def _hr(t: str) -> None:
    print("\n" + "=" * 74)
    print(t)
    print("=" * 74)


def _manifest(path: str) -> None:
    p = path[:-6] + ".manifest.json" if path.endswith(".jsonl") else None
    if not p or not os.path.exists(p):
        print("⚠ 매니페스트가 없다 — 이 로그가 어느 상자·어느 커밋에서 나왔는지 모른다")
        return
    with open(p, encoding="utf-8") as f:
        m = json.load(f)
    g = m.get("git") or {}
    print(f"  상자        {m.get('box_name')}   seed {m.get('seed')}   "
          f"n {m.get('n')} × n_ser {m.get('n_ser')}")
    print(f"  커밋        {g.get('commit')}{' (dirty)' if g.get('dirty') else ''}"
          f"   constants {m.get('constants_sha256')}")
    print(f"  실행        {m.get('created')} → {m.get('finished')}   "
          f"워커 {m.get('workers')}")
    for a in space.AXES:
        b = (m.get("box") or {}).get(a)
        if b:
            print(f"    {a:14s} {b[0]:12.5f} – {b[1]:<12.5f}")


def _mixed(rows: list) -> None:
    """한 파일에 여러 실행이 섞였는지 — id 는 실행 안에서만 유일하다."""
    runs = Counter(r.get("run") or "(도장 없음)" for r in rows)
    ids = Counter((r.get("run"), r["id"]) for r in rows)
    dup = sum(1 for c in ids.values() if c > 1)
    if len(runs) > 1 or dup:
        print("\n⚠ 이 로그에 여러 실행이 섞여 있다.")
        for name, c in runs.most_common():
            print(f"    {name:28s} {c:6d} 행")
        if dup:
            print(f"    같은 (실행, id) 가 {dup} 쌍 겹친다 — 아래 통계는 그만큼 이중계산된다.")
        print("    실행별로 갈라 보려면 --out 을 나눠 다시 돌린다.")


def _outcomes(rows: list) -> None:
    _hr("1. 표본 결과 — 유효 표본 비율과 탈락 사유")
    n = len(rows)
    ok = sum(1 for r in rows if r["feasible"])
    print(f"  표본 {n} 점 (복제 포함)   합격 {ok} 점   유효율 {ok / max(n, 1):.1%}")
    print()
    codes = Counter(r["fail_code"] or "OK" for r in rows)
    for code, c in codes.most_common():
        bar = "#" * int(40 * c / max(n, 1))
        print(f"  {code:30s} {c:6d}  {c / max(n, 1):6.1%}  {bar}")
    print("\n  사유 코드는 DOE 로그의 1급 데이터다 (ICD §5) — 설정 탓과 물리 탓을")
    print("  한 색으로 칠하지 않는다. exception: 으로 시작하는 것은 물리 탈락이")
    print("  아니라 코드가 죽은 것이므로 따로 봐야 한다.")


def _gates(rows: list) -> None:
    _hr("2. 게이트 — 어디서 걸리나")
    vals = defaultdict(list)
    for r in rows:
        for name, v in (r.get("g") or {}).items():
            if isinstance(v, (int, float)):
                vals[name].append(v)

    def table(names, title, note):
        print(f"\n  [{title}] {note}")
        print(f"    {'게이트':8s} {'표본':>6s} {'합격률':>8s} {'최소':>12s} "
              f"{'중앙':>12s}")
        for g in names:
            v = vals.get(g) or []
            if not v:
                print(f"    {g:8s} {'—':>6s}   (계산된 표본 없음)")
                continue
            passed = sum(1 for x in v if x >= 0)
            print(f"    {g:8s} {len(v):6d} {passed / len(v):8.1%} "
                  f"{min(v):12.4f} {st.median(v):12.4f}")

    table(SIZING_GATES, "사이징 게이트", "잔차다 — 0 에 붙는 게 정상 (docs §11-44)")
    table(QUALITY_GATES, "품질 게이트", "실제 스크리닝은 여기서 일어난다")


def _ec(rows: list) -> None:
    _hr("3. 성적표 — 합격 표본의 분포")
    good = [r for r in rows if r["feasible"] and r.get("ec")]
    if not good:
        print("  합격 표본이 없다.")
        return
    names = list(good[0]["ec"].keys())
    print(f"    {'EC':24s} {'최소':>14s} {'중앙':>14s} {'최대':>14s}")
    for n in names:
        v = [r["ec"][n] for r in good if isinstance(r["ec"].get(n), (int, float))]
        if v:
            print(f"    {n:24s} {min(v):14.4f} {st.median(v):14.4f} {max(v):14.4f}")
    print("\n  ⚠ EC 가중합 점수(§6.2)는 **미구현**이다. 단위가 제각각이라 정규화가")
    print("    필요한데 기준이 아직 없다 — 표본 최소–최대로 할지, 요구조건 기준으로")
    print("    할지에 따라 순위가 뒤집힌다. [확정 필요]")


def _axes(rows: list) -> None:
    _hr("4. 축별 — 어느 변수가 합격을 지배하나 (4분위 합격률)")
    n = len(rows)
    if n < 40:
        print(f"  표본 {n} 점 — 4분위 합격률을 낼 표본이 못 된다 (40 점 이상).")
        return
    # 판정을 붙이려면 구간당 표본이 충분해야 한다. 구간당 50 점이면 합격률의
    # 표준오차가 7 %p 정도라 구간 간 50 %p 차이를 신호로 읽어도 된다. 그 아래에서는
    # 숫자만 찍고 판정을 붙이지 않는다 — 잡음에 이름을 붙이면 그게 결론이 된다.
    judge = n >= 200
    print("    각 축을 표본의 4분위로 나눠 그 구간의 합격률을 본다. 한쪽 끝만")
    print("    합격률이 낮으면 상자가 그 방향으로 너무 넓다는 뜻이다.")
    if not judge:
        print(f"    ⚠ 표본 {n} 점 — 구간당 {n // 4} 점뿐이라 **판정은 생략한다.**")
    print()
    print(f"    {'축':14s} {'Q1':>8s} {'Q2':>8s} {'Q3':>8s} {'Q4':>8s}   판정")
    for a in space.AXES:
        key = (lambda r: r["dv"]["x_fin"] / (r["dv"]["d_body"] * r["dv"]["lambda_body"])
               ) if a == "x_fin_ratio" else (lambda r, a=a: r["dv"][a])
        s = sorted(rows, key=key)
        q = [s[i * n // 4:(i + 1) * n // 4] for i in range(4)]
        rate = [sum(1 for r in b if r["feasible"]) / max(len(b), 1) for b in q]
        spread = max(rate) - min(rate)
        mark = ("" if not judge else
                "지배" if spread >= 0.5 else "반응" if spread >= 0.2 else "")
        print(f"    {a:14s}" + "".join(f"{x:8.0%}" for x in rate) + f"   {mark}")


def _purity(rows: list) -> None:
    _hr("5. 순수성 — 같은 설계점이 같은 답을 냈나")
    # (실행, id) 로 짝을 찾는다 — id 만으로 찾으면 다른 실행의 행과 짝지어져
    # 순수성 판정이 거짓으로 깨진다.
    by_key = {(r.get("run"), r["id"]): r for r in rows}
    pairs = [(r, by_key[(r.get("run"), r["dup_of"])]) for r in rows
             if r.get("dup_of") is not None
             and (r.get("run"), r["dup_of"]) in by_key]
    if not pairs:
        print("  복제 쌍이 없다 (--dup-frac 0 으로 돌렸다).")
        return
    bad = [x["id"] for x, y in pairs if row.signature(x) != row.signature(y)]
    if bad:
        print(f"  FAIL — {len(bad)}/{len(pairs)} 쌍이 갈라졌다 (id {bad[:8]}).")
        print("  같은 MTOW 가 같은 응답질량을 안 냈다는 뜻이다 — 캐시·전역·난수를")
        print("  의심한다 (CLAUDE.md 규칙 3).")
    else:
        print(f"  OK — 복제 {len(pairs)} 쌍이 전부 비트 동일하다.")
        print("  병렬 실행에서 설계점끼리 오염되지 않았다.")


def _timing(rows: list) -> None:
    _hr("6. 시간")
    w = [r["wall_s"] for r in rows if isinstance(r.get("wall_s"), (int, float))]
    if not w:
        return
    okw = [r["wall_s"] for r in rows if r["feasible"]]
    print(f"  설계점당 (직렬 환산)  중앙 {st.median(w):.3f} s   "
          f"최소 {min(w):.3f}   최대 {max(w):.3f}")
    if okw:
        print(f"  끝까지 간 점만        중앙 {st.median(okw):.3f} s")
    print(f"  누적 CPU 시간         {sum(w) / 60:.1f} 분")
    print("\n  탈락점은 앞구획에서 끊기므로 훨씬 싸다 — 유효율이 낮은 상자는")
    print("  겉보기 평균 시간도 같이 낮아진다. 규모 환산은 합격점 중앙값으로 한다.")


def main_cli(argv=None) -> int:
    stdout_utf8()
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print("사용법: python -m doe.report <로그.jsonl>")
        return 2
    path = argv[0]
    rows = load(path)
    if not rows:
        print(f"{path} 에 읽을 행이 없다.")
        return 2
    _hr(f"DOE 보고 — {path}")
    _manifest(path)
    _mixed(rows)
    _outcomes(rows)
    _gates(rows)
    _ec(rows)
    _axes(rows)
    _purity(rows)
    _timing(rows)
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main_cli())
