"""P7 통합 게이트 — DOE 전 필수 측정.

TASKS.md P7 이 요구하는 네 항목을 재현 가능한 형태로 측정한다.

    1. 사이징 루프 수렴 횟수와 성장계수 Ŝ (+ 분해)
    2. 설계점당 실행 시간 · 모듈별 시간 점유율
    3. 설계변수를 흔들어 결과가 연속적인지
    4. 같은 설계점 2회 실행이 동일한지 (순수성)

실행:  python -m validation.p7_gate            (전체, 약 7 분)
       python -m validation.p7_gate --quick    (연속성 스윕 축소)

**이건 검증이 아니라 계측이다.** 물리가 맞는지는 말하지 않는다.
"""
from __future__ import annotations

import cProfile
import dataclasses
import pstats
import sys
import time
from collections import defaultdict

import main
from interfaces import DesignVars

# 대표 설계점 — main.py 의 것과 같아야 한다 (g1~g9 전부 통과하는 점)
DV0 = DesignVars(d_body=0.09, lambda_body=8.0, S_fin=0.030, x_fin=0.60, AR_fin=2.2,
                 f_mount=1.0, n_design=4.0, d_prop=0.13, pd_prop=1.50, n_ser=6,
                 k_E=1.0, k_mot=1.0)

# 연속성 스윕 — 정수 변수(n_ser)는 뺀다.
SWEEP_VARS = ["d_body", "lambda_body", "S_fin", "x_fin", "AR_fin", "f_mount",
              "n_design", "d_prop", "pd_prop", "k_E", "k_mot"]
SWEEP_SPAN = 0.04          # ±4 %
SWEEP_N = 9

# 사이징이 0 으로 몰아붙이는 잔차 — 품질 게이트와 섞어 읽으면 안 된다
SIZING_GATES = ("g2", "g3", "g4")


def _dv(**over) -> DesignVars:
    return dataclasses.replace(DV0, **over)


def _hr(title: str) -> None:
    print("\n" + "=" * 74)
    print(title)
    print("=" * 74)


def _parts_fn(dv: DesignVars):
    """설계점 하나에 대한 resp_parts_of 를 꺼낸다 — evaluate 와 같은 조립이다.

    런처가 클로저로 만들어 growth_split 에 넘기는 것을 그대로 잡아 온다.
    여기서 다시 조립하면 식이 복제된다.
    """
    holder = {}
    orig = main.wght.growth_split

    def spy(history, resp_parts_of):
        holder["f"] = resp_parts_of
        return orig(history, resp_parts_of)

    main.wght.growth_split = spy
    try:
        main.evaluate(dv)
    finally:
        main.wght.growth_split = orig
    return holder["f"]


# ══════════════════════════════════════════════════════════════════════════
# 1. 수렴과 성장계수
# ══════════════════════════════════════════════════════════════════════════
def part1_convergence():
    _hr("1. 사이징 루프 수렴과 성장계수")
    r = main.evaluate(DV0)
    d = r.diag
    M0 = r.ec["C1_MTOW[kg]"]

    print("  상태          {}".format(d["wght_status"]))
    print("  반복 횟수      {}".format(d["n_iter"]))
    print("  잔차          {:.3e}".format(d["err"]))
    print("  MTOW          {:.6f} kg".format(M0))
    print("  S_hat (WGHT)  {:+.6f}   <- 수렴 근방 수축비에서 뽑은 국소 추정"
          .format(d["S_hat"]))

    sp = d["S_split"]
    tot = sum(v for v in sp.values() if v is not None)
    print("")
    print("  S_hat 분해 (할선):")
    for name, label in (("struct", "구조"), ("motor", "모터"), ("batt", "배터리")):
        share = 100.0 * sp[name] / tot if tot else 0.0
        print("    {:<8}{:+.6f}   ({:5.1f} %)".format(label, sp[name], share))
    print("    {:<8}{:+.6f}".format("합", tot))

    ratio = abs(tot / d["S_hat"]) if d["S_hat"] else float("nan")
    print("")
    print("  주의: WGHT 의 S_hat 과 분해 합이 {:.1f}배 어긋난다.".format(ratio))
    print("  같은 이름이지만 다른 추정량이다 — WGHT 것은 수렴 근방 수축비에서 뽑은")
    print("  국소 추정, 분해는 유한 구간의 할선이다. 아래에서 구간 폭을 바꿔 가며 잰다.")

    print("")
    print("  구간 폭별 할선 (수렴점 {:.4f} kg 중심):".format(M0))
    print("    {:>8}{:>12}{:>12}{:>12}{:>12}".format("폭", "구조", "모터", "배터리", "합"))
    parts_of = _parts_fn(DV0)
    for h in (0.002, 0.01, 0.05, 0.20):
        pa, pb = parts_of(M0 * (1 - h)), parts_of(M0 * (1 + h))
        dM = 2.0 * M0 * h
        s = {n: (pb[n] - pa[n]) / dM for n in pa}
        print("    {:6.1f} %{:12.6f}{:12.6f}{:12.6f}{:12.6f}".format(
            h * 100, s["struct"], s["motor"], s["batt"], sum(s.values())))
    print("  구조 항이 구간에 따라 흔들리면 w_fill 계단 때문이다 (로컬 개정 11-39).")
    return M0


# ══════════════════════════════════════════════════════════════════════════
# 2. 실행 시간과 모듈 점유율
# ══════════════════════════════════════════════════════════════════════════
def part2_timing(n_rep: int = 5) -> float:
    _hr("2. 설계점당 실행 시간과 모듈별 점유율")
    ts = []
    for _ in range(n_rep):
        t0 = time.perf_counter()
        main.evaluate(DV0)
        ts.append(time.perf_counter() - t0)
    ts.sort()
    med = ts[len(ts) // 2]
    print("  {} 회 측정 - 중앙값 {:.3f} s  (최소 {:.3f} / 최대 {:.3f})".format(
        n_rep, med, ts[0], ts[-1]))

    # Ŝ 분해는 resp_of 를 2 회 더 부른다 — DOE 에서는 끌 수 있어야 한다
    t0 = time.perf_counter()
    for _ in range(3):
        main.evaluate(DV0, split=False)
    off = (time.perf_counter() - t0) / 3.0
    print("  split=False (Ŝ 분해 끔)  {:.3f} s   -> 분해 비용 {:+.0f} %".format(
        off, 100.0 * (med - off) / off))

    print("")
    print("  DOE 규모 환산 (단일 프로세스, split=False 기준):")
    for n in (100, 1_000, 10_000, 50_000):
        sec = n * off
        extra = "  ({:.1f} 시간)".format(sec / 3600) if sec > 3600 else ""
        print("    {:>6,} 점 -> {:8.1f} 분{}".format(n, sec / 60, extra))

    pr = cProfile.Profile()
    pr.enable()
    main.evaluate(DV0)
    pr.disable()
    stats = pstats.Stats(pr).stats
    by_mod = defaultdict(float)
    for (fn, _ln, _nm), (_cc, _nc, tt, _ct, _cal) in stats.items():
        f = str(fn).replace("\\", "/")
        if "/modules/" in f:
            mod = "modules/" + f.rsplit("/", 1)[-1]
        elif f.endswith("main.py"):
            mod = "main.py (런처)"
        elif f.endswith("constants.py") or f.endswith("interfaces.py"):
            mod = "계약·상수"
        elif "numpy" in f or "scipy" in f:
            mod = "numpy/scipy"
        else:
            mod = "기타 (표준·내장)"
        by_mod[mod] += tt
    total = sum(by_mod.values())
    print("")
    print("  모듈별 점유율 (tottime, 합 {:.3f} s):".format(total))
    for mod, tt in sorted(by_mod.items(), key=lambda x: -x[1]):
        if tt / total < 0.002:
            continue
        bar = "#" * int(round(40 * tt / total))
        print("    {:<22}{:5.1f} %  {:7.3f} s  {}".format(
            mod, 100 * tt / total, tt, bar))

    # 뜨거운 함수 — DOE 비용을 줄일 지렛대가 어디 있는지 본다
    rows = []
    for (fn, ln, nm), (_cc, nc, tt, _ct, _cal) in stats.items():
        f = str(fn).replace("\\", "/")
        if "/modules/" in f or f.endswith("main.py"):
            rows.append((tt, nc, f.rsplit("/", 1)[-1], nm))
    rows.sort(reverse=True)
    print("")
    print("  뜨거운 함수 상위 8:")
    print("    {:>8}{:>12}  {:<14}{}".format("tottime", "호출수", "파일", "함수"))
    for tt, nc, f, nm in rows[:8]:
        print("    {:8.3f}{:12,}  {:<14}{}".format(tt, nc, f, nm))
    return med


# ══════════════════════════════════════════════════════════════════════════
# 3. 게이트 여유
# ══════════════════════════════════════════════════════════════════════════
def part3_gates() -> None:
    _hr("3. g1-g9 — 어디가 빡빡한가")
    r = main.evaluate(DV0)
    print("  판정: {}".format("합격" if r.feasible else r.fail_code))
    print("")

    # g2·g3·g4 는 사이징이 0 으로 몰아붙이는 잔차다. size_motor 는 트림·팁마하·열을
    # 동시에 만족하는 최소 질량을 찾은 뒤 k_mot 을 곱하므로, k_mot=1.0 에서 구속
    # 조건은 정의상 0 에 붙는다. 여유가 작다고 "빡빡한 설계점"인 게 아니다.
    print("  [사이징 잔차] — 0 에 붙는 것이 정상이다. 스크리닝 정보가 아니다.")
    for name in SIZING_GATES:
        print("    {:<5}{:>+12.4f}".format(name, r.g[name]))
    print("    구속 조건이 g3 이면, 그 여유를 사는 손잡이는 k_mot 이다.")

    print("")
    print("  [품질 게이트] — 실제 스크리닝은 여기서 일어난다.")
    qual = sorted(((n, v) for n, v in r.g.items()
                   if n not in SIZING_GATES and n != "g1"), key=lambda x: x[1])
    for rank, (name, v) in enumerate(qual, 1):
        mark = "  <- 가장 빡빡" if rank == 1 else ("  <- 2번째" if rank == 2 else "")
        print("    {:<5}{:>+12.4f}   {}{}".format(name, v, rank, mark))
    print("")
    print("  [순항 성립] g1 {:+.4f} — ⓪ 에서 거르는 조건이라 여유가 크다."
          .format(r.g["g1"]))


def part3b_sizing_residual() -> None:
    """g2·g3 가 정말 구조적으로 0 에 붙는지 여러 설계점에서 확인한다."""
    _hr("3b. 사이징 잔차가 구조적인가 — 설계점을 바꿔 가며")
    print("  {:<24}{:>10}{:>10}   구속 조건".format("변형", "g2", "g3"))
    cases = [("공칭", {}), ("d_prop 0.12", {"d_prop": 0.12}),
             ("d_prop 0.15", {"d_prop": 0.15}), ("n_ser 5", {"n_ser": 5}),
             ("n_ser 7", {"n_ser": 7}), ("d_body 0.10", {"d_body": 0.10}),
             ("pd_prop 1.7", {"pd_prop": 1.7})]
    for name, over in cases:
        r = main.evaluate(_dv(**over), split=False)
        g2, g3 = r.g.get("g2"), r.g.get("g3")
        if g2 is None or g3 is None:
            print("  {:<24}{:>10}{:>10}   {}".format(name, "-", "-", r.fail_code))
            continue
        bind = "열(g3)" if abs(g3) < abs(g2) else "팁마하(g2)"
        print("  {:<24}{:>+10.4f}{:>+10.4f}   {}".format(name, g2, g3, bind))
    print("")
    print("  g3 가 어디서나 0 에 붙으면 열이 항상 구속 조건이라는 뜻이다.")
    print("  그러면 g3 는 스크리닝 변수가 아니라 사이징의 출력이다.")


# ══════════════════════════════════════════════════════════════════════════
# 4. 연속성
# ══════════════════════════════════════════════════════════════════════════
def part4_continuity(quick: bool = False) -> list:
    _hr("4. 연속성 — 설계변수를 흔들면 결과가 매끄럽게 변하나")
    n = 5 if quick else SWEEP_N
    print("  각 변수를 공칭 +-{:.0f} % 에서 {} 점 스윕.".format(SWEEP_SPAN * 100, n))
    print("  C1 의 인접 표본 간 차분을 보고, 게이트가 가장 크게 움직인 것도 함께 낸다.")
    print("")
    print("  {:<13}{:>8}{:>11}  {:<11}{:<14}{}".format(
        "변수", "점프비", "C1 진폭", "판정", "최대변동 게이트", "탈락"))
    flagged = []
    for var in SWEEP_VARS:
        base = getattr(DV0, var)
        c1s, gs, nfail = [], [], 0
        for i in range(n):
            f = 1.0 - SWEEP_SPAN + 2.0 * SWEEP_SPAN * i / (n - 1)
            rr = main.evaluate(_dv(**{var: base * f}), split=False)
            if not rr.feasible:
                nfail += 1
            v = rr.ec.get("C1_MTOW[kg]")
            if v is None:
                continue          # 게이트 탈락으로 조기 종료 — 예외가 아니다
            c1s.append(v)
            gs.append(rr.g)
        if len(c1s) < 3:
            print("  {:<13}{:>8}{:>11}  {:<11}{:<14}{}/{}".format(
                var, "-", "-", "표본 부족", "-", nfail, n))
            continue

        amp = max(c1s) - min(c1s)
        dd = sorted(abs(c1s[i + 1] - c1s[i]) for i in range(len(c1s) - 1))
        med, mx = dd[len(dd) // 2], dd[-1]

        # C1 자체가 안 움직이면 계단이 아니라 **무반응**이다. 구분하지 않으면
        # 0/0 이 inf 로 나와 멀쩡한 변수가 계단으로 오진된다.
        if amp < 1e-9:
            verdict, ratio_s = "C1 무반응", "-"
        elif med < 1e-12:
            verdict, ratio_s = "**계단**", "inf"
            flagged.append((var, "계단"))
        else:
            ratio = mx / med
            ratio_s = "{:.2f}".format(ratio)
            verdict = ("매끈" if ratio < 3.0
                       else ("계단 의심" if ratio < 10 else "**계단**"))
            if ratio >= 3.0:
                flagged.append((var, verdict))

        # 게이트 중 가장 크게 움직인 것 — C1 이 안 움직여도 설계 영향은 클 수 있다
        best, bswing = "-", 0.0
        for gname in gs[0]:
            sw = max(g[gname] for g in gs) - min(g[gname] for g in gs)
            if sw > bswing:
                best, bswing = gname, sw
        print("  {:<13}{:>8}{:>11.3e}  {:<11}{:<14}{}/{}".format(
            var, ratio_s, amp, verdict,
            "{} {:.3f}".format(best, bswing), nfail, n))

    print("")
    if flagged:
        print("  계단이 잡힌 변수: " +
              ", ".join("{}({})".format(v, w) for v, w in flagged))
        print("  구조의 w_fill 양자화(로컬 개정 11-39)가 1순위 후보다 — 계단 743 mg.")
    else:
        print("  전 변수에서 매끈하다.")
    print("  'C1 무반응' 은 결함이 아니다 — 질량에 안 걸리는 변수라는 뜻이며,")
    print("  그런 변수도 게이트(예: x_fin -> g8)에는 크게 작용할 수 있다.")
    return flagged


# ══════════════════════════════════════════════════════════════════════════
# 5. 순수성
# ══════════════════════════════════════════════════════════════════════════
def part5_purity() -> bool:
    _hr("5. 순수성 — 같은 설계점이 항상 같은 값을 내나")

    def sig(r):
        return (tuple(sorted(r.ec.items())), tuple(sorted(r.g.items())), r.fail_code)

    a = sig(main.evaluate(DV0))
    b = sig(main.evaluate(DV0))
    main.evaluate(_dv(d_body=0.10, lambda_body=7.0, S_fin=0.036, x_fin=0.52))
    c = sig(main.evaluate(DV0))
    d = sig(main.evaluate(DV0, split=False))
    print("  같은 설계점 2회 연속        {}".format("OK - 비트 동일" if a == b else "FAIL"))
    print("  사이에 다른 설계점 삽입      {}".format("OK - 비트 동일" if a == c else "FAIL"))
    print("  split 끄고 켜기             {}".format("OK - 비트 동일" if a == d else "FAIL"))
    ok = (a == b == c == d)
    if ok:
        print("")
        print("  resp_of 순수성 유지 - 캐시·전역·난수 없음 (CLAUDE.md 규칙 3).")
    return ok


if __name__ == "__main__":
    quick = "--quick" in sys.argv
    t0 = time.perf_counter()
    part1_convergence()
    med = part2_timing()
    part3_gates()
    part3b_sizing_residual()
    flagged = part4_continuity(quick)
    ok = part5_purity()
    _hr("P7 게이트 요약")
    print("  설계점당      {:.3f} s  (split=True)".format(med))
    print("  순수성        {}".format("OK" if ok else "FAIL"))
    print("  연속성        {}".format(
        "전부 매끈" if not flagged else "{} 개 변수에서 계단".format(len(flagged))))
    print("  총 소요       {:.1f} s".format(time.perf_counter() - t0))
