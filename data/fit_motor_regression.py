"""모터 회귀 적합 — ICD0-008 §8 A-2

실행: python -m data.fit_motor_regression   (저장소 루트에서)

data/motor_specs.csv 를 읽어 아래 두 회귀를 로그-선형 최소제곱으로 적합하고,
constants.py 에 넣을 계수와 **적합 근거**(R²·행별 잔차·유효 범위)를 함께 찍는다.

    R_mot = a_R  · m_mot[kg]^b_R · kv^c_R      [Ω]   (상간 기준)
    I0    = a_I0 · kv^b_I0                     [A]

표준 라이브러리만 쓴다 — 조사 자산이 파이프라인에 없는 의존성을 끌어들이지 않는다.

⚠ 이 적합은 **조사 범위 안에서만 유효**하다. 회귀는 외삽에서 무효라는 §8 A-2 의
  경고가 여기 그대로 적용된다. 아래 출력의 '유효 범위' 를 벗어난 (m_mot, kv) 를
  motor_regression() 에 넣으면 값을 믿으면 안 된다.
"""
import csv
import math
import os

CSV = os.path.join(os.path.dirname(__file__), "motor_specs.csv")

# I0 적합에서 뺄 행 — 근거는 motor_specs.csv 의 note 열
I0_OUTLIERS = {"DarwinFPV 2207 2400KV"}


def load(path=CSV):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            rows.append(line)
    for r in csv.DictReader(rows):
        yield {
            "name": r["name"],
            "m": float(r["m_g"]) / 1000.0,                       # [kg]
            "kv": float(r["kv"]),
            "R": float(r["R_mohm"]) / 1000.0 if r["R_mohm"] else None,   # [Ω]
            "I0": float(r["I0_A_10V"]) if r["I0_A_10V"] else None,
        }


def loglin(rows, ykey, xkeys):
    """log y = log a + Σ b_i·log x_i. 정규방정식을 가우스 소거로 직접 푼다."""
    n, p = len(rows), len(xkeys) + 1
    A = [[1.0] + [math.log(r[k]) for k in xkeys] for r in rows]
    y = [math.log(r[ykey]) for r in rows]
    ATA = [[sum(A[k][i] * A[k][j] for k in range(n)) for j in range(p)] for i in range(p)]
    ATy = [sum(A[k][i] * y[k] for k in range(n)) for i in range(p)]
    for i in range(p):
        pv = max(range(i, p), key=lambda r: abs(ATA[r][i]))
        ATA[i], ATA[pv] = ATA[pv], ATA[i]
        ATy[i], ATy[pv] = ATy[pv], ATy[i]
        for r in range(i + 1, p):
            f = ATA[r][i] / ATA[i][i]
            for c in range(i, p):
                ATA[r][c] -= f * ATA[i][c]
            ATy[r] -= f * ATy[i]
    b = [0.0] * p
    for i in reversed(range(p)):
        b[i] = (ATy[i] - sum(ATA[i][j] * b[j] for j in range(i + 1, p))) / ATA[i][i]
    yh = [sum(b[j] * A[k][j] for j in range(p)) for k in range(n)]
    ym = sum(y) / n
    r2 = 1 - sum((y[k] - yh[k]) ** 2 for k in range(n)) / sum((v - ym) ** 2 for v in y)
    res = [(math.exp(yh[k]) / math.exp(y[k]) - 1) * 100 for k in range(n)]
    return math.exp(b[0]), b[1:], r2, res


def corr(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((v - mx) ** 2 for v in xs))
    sy = math.sqrt(sum((v - my) ** 2 for v in ys))
    return sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / (sx * sy)


def report(rows, res, label):
    a, b, r2, err = res
    print(f"\n[{label}]  R² = {r2:.4f}   최대 |잔차| = {max(abs(e) for e in err):.0f}%")
    for r, e in zip(rows, err):
        print(f"    {r['name']:<34} {e:+6.0f}%")
    return a, b


def main():
    data = list(load())
    print("=" * 78)
    print(f"모터 회귀 적합 — 표본 {len(data)}종")
    print("=" * 78)
    print(f"질량 {min(d['m'] for d in data)*1e3:.1f} – {max(d['m'] for d in data)*1e3:.1f} g"
          f"   ·   kv {min(d['kv'] for d in data):.0f} – {max(d['kv'] for d in data):.0f} rpm/V")

    # ── R_mot ──
    rR = [d for d in data if d["R"]]
    rho = corr([math.log(d["m"]) for d in rR], [math.log(d["kv"]) for d in rR])
    print(f"\nlog(m)–log(kv) 상관 = {rho:+.3f}")
    if abs(rho) > 0.7:
        print("  ⚠ 공선성이 강하다. 지수 b_R·c_R 를 **개별로** 해석하면 안 된다 —")
        print("    표본에서 작은 모터일수록 kv 가 높아 두 축이 얽혀 있다.")
        print("    예측값은 조사 범위 안에서만 신뢰할 수 있다.")

    a_R, (b_R, c_R) = report(rR, loglin(rR, "R", ["m", "kv"]),
                             f"R_mot = a·m^b·kv^c   ({len(rR)}행)")

    # ── I0 ──
    rI_all = [d for d in data if d["I0"]]
    rI = [d for d in rI_all if d["name"] not in I0_OUTLIERS]
    dropped = [d["name"] for d in rI_all if d["name"] in I0_OUTLIERS]
    _, _, r2_all, _ = loglin(rI_all, "I0", ["kv"])
    a_I0, (b_I0,) = report(rI, loglin(rI, "I0", ["kv"]),
                           f"I0 = a·kv^b   ({len(rI)}행, 이상치 {len(dropped)}행 제외)")
    print(f"    이상치 제외 전 R² = {r2_all:.4f} → 제외 후 개선. 제외: {', '.join(dropped)}")

    _, b2, r2_2, _ = loglin(rI, "I0", ["m", "kv"])
    print(f"    참고: 질량을 넣으면 R²={r2_2:.4f} 로 오르지만(b_m={b2[0]:+.3f}, b_kv={b2[1]:+.3f})")
    print(f"          위 공선성 때문에 지수 분해가 불안정해 ICD 가 정한 kv 단독 형태를 유지한다.")

    print("\n" + "=" * 78)
    print("constants.py 에 넣을 값")
    print("=" * 78)
    print(f"a_R, b_R, c_R = {a_R:.6g}, {b_R:.6g}, {c_R:.6g}")
    print(f"a_I0, b_I0 = {a_I0:.6g}, {b_I0:.6g}")
    print(f"\n유효 범위 (이 밖은 외삽 — 값을 믿지 말 것):")
    print(f"  m_mot : {min(d['m'] for d in rR)*1e3:.1f} – {max(d['m'] for d in rR)*1e3:.1f} g")
    print(f"  kv    : {min(d['kv'] for d in rR):.0f} – {max(d['kv'] for d in rR):.0f} rpm/V")
    print("\n⚠ 출처가 전부 2차(판매처·검색 결과)다. 제조사 데이터시트로 재확인 필요.")
    print("⚠ R 이 상간(phase-to-phase)이라고 가정했다. 상당이면 전부 2배 → 동손 2배.")


if __name__ == "__main__":
    from common.out import stdout_utf8
    stdout_utf8()
    main()
