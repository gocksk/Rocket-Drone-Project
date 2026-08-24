"""실측 도구 — resid_floor 와 양자화 크기

실행: python3 -m validation.wght.measure_noise   (저장소 루트에서)

[왜 실측인가]
constants.py 의 resid_floor = 1e-6 은 근거 없는 잠정값이다. 배정도 반올림은 상대 1e-16
수준이라 10 자릿수 위이고, 정작 잡아야 할 'STRC 자체의 수치 노이즈'보다 큰지 작은지 모른다.

[측정 원리]
resid = m_fixed + m_str(MTOW) - MTOW 에서 m_fixed 와 MTOW 는 정확한 값이다.
따라서 resid 의 불확실성은 곧 m_str 의 불확실성이다. MTOW 를 아주 좁게 훑으면서
m_str 의 응답을 보면 두 가지가 한 번에 나온다:

  · 노이즈  — 국소 선형 추세를 뺀 잔차의 산포. resid_floor 는 이보다 커야 한다.
  · 양자화  — 고유값이 이산 격자를 이루면 그 최소 간격. limit_cycle 판정용.

[해석 주의]
매끄러운 모델이면 노이즈≈0, 양자화 없음으로 나온다. 그건 '측정 실패'가 아니라
'그 모델은 하한 가드가 거의 필요 없다'는 결과다. 실제 STRC 는 슬라이서 회귀
(k_sl_*)가 들어오면 계단형일 수 있으므로, 계수가 실측으로 교체되면 반드시 다시 잰다.

표준 라이브러리만 쓴다 — 파이프라인에 없는 의존성을 검증 자산이 끌어들이지 않는다.

⚠ ICD0-008 §8 C-4 미완 — 측정 대상이 구조 무게에서 **전체 응답 질량**(구조+모터+
  배터리)으로 넓어져야 하는데, 지금은 PROP·MISS 가 스텁이라 구조 항만 잰다.
  P4·P5 가 끝나고 dt_miss 가 확정되면 resp_of 전체를 물려 다시 재야 한다.
"""
import math

import constants as k
import main
from interfaces import DesignVars
from modules import strc
from validation.wght.strc_stub import (make_linear_strc, make_quantized_strc,
                                       make_power_strc, make_noisy_strc)


def probe(resp_of, MTOW_center, rel_window=1e-3, n=401):
    """MTOW_center 주변 ±rel_window 를 n 점으로 훑어 응답 질량을 얻는다.

    창을 좁게(기본 0.1%) 잡는 이유: 넓으면 모델의 '진짜 기울기'가 섞여 들어와
    노이즈와 구분되지 않는다. 수렴 근방에서 반복이 실제로 움직이는 폭이 이 정도다.
    """
    xs = [MTOW_center * (1.0 + rel_window * (2.0 * i / (n - 1) - 1.0)) for i in range(n)]
    ys = [resp_of(x)[0] for x in xs]
    return xs, ys


def measure_quantum(ys, rel_tol=1e-12):
    """고유값이 이산 격자면 최소 간격을 돌려준다. 연속이면 None.

    판정: 표본 수 대비 고유값이 충분히 적으면 '계단형'으로 본다.
          매끄러운 함수는 표본마다 값이 달라 고유값 수 ≈ 표본 수가 된다.
    """
    scale = max(1.0, max(abs(y) for y in ys))
    step = scale * rel_tol
    uniq = sorted({round(y / step) * step for y in ys})
    if len(uniq) >= len(ys) * 0.5:          # 절반 이상이 서로 다르면 연속으로 본다
        return None, len(uniq)
    if len(uniq) < 2:
        return 0.0, len(uniq)               # 창 안에서 완전히 평평 (한 계단 안)
    return min(b - a for a, b in zip(uniq, uniq[1:])), len(uniq)


def measure_noise(xs, ys):
    """국소 선형 추세를 뺀 잔차의 표준편차 = 수치 노이즈.

    1차 적합을 쓰는 이유: 좁은 창 안에서 매끄러운 모델은 거의 직선이므로,
    직선을 빼고 남는 것이 곧 '모델이 만들어내는 흔들림'이다.
    """
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx if sxx > 0 else 0.0
    intercept = my - slope * mx
    res = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    mr = sum(res) / n
    return math.sqrt(sum((r - mr) ** 2 for r in res) / n), slope


def measure(resp_of, MTOW_center, name='', rel_window=1e-3, n=401, k_sigma=3.0,
            max_widen=8):
    """한 모델에 대한 전체 측정. resid_floor 권고값까지 낸다.

    창이 양자보다 좁으면 표본이 전부 한 계단 안에 들어가 고유값이 1개가 되고,
    그러면 '양자 크기 0'이라는 무의미한 답이 나온다. 그 권고값을 그대로 쓰면
    하한 가드가 통째로 꺼진다. 그래서 고유값이 2개 이상 보일 때까지 창을 배로 넓힌다.
    """
    widened = 0
    while True:
        xs, ys = probe(resp_of, MTOW_center, rel_window, n)
        quantum, n_uniq = measure_quantum(ys)
        if n_uniq >= 2 or widened >= max_widen:
            break
        rel_window *= 2.0          # 한 계단 안에 갇혔다 — 창을 넓힌다
        widened += 1

    noise, slope = measure_noise(xs, ys)
    staircase = quantum is not None and quantum > 0.0

    # resid_floor 는 '의미 없는 차이를 신호로 착각하지 않을' 크기여야 한다.
    #  · 계단형이면 양자의 절반. 그보다 작은 resid 차이는 물리적으로 존재하지 않는다.
    #    이때 선형적합 잔차는 '노이즈'가 아니라 계단 자체이므로 floor 산정에 쓰지 않는다.
    #  · 매끄러우면 노이즈의 k_sigma 배.
    floor_abs = quantum * 0.5 if staircase else k_sigma * noise
    return {'name': name, 'MTOW_center': MTOW_center, 'slope_local': slope,
            'noise_abs': noise, 'staircase': staircase, 'quantum': quantum,
            'n_unique': n_uniq, 'rel_window': rel_window, 'widened': widened,
            'floor_abs': floor_abs, 'floor_rel': floor_abs / MTOW_center}


def real_strc_case():
    """실제 modules/strc.py 를 측정 대상으로 만든다 — 이게 이 도구의 본래 용도다.

    ⓪ 전처리는 main.preprocess() 하나만 부른다 (§8 C-1 단일 출처).
    ⚠ 응답 질량 중 구조 항만 물린다 — 모터·배터리는 아직 스텁이다 (§8 C-4 미완).
    """
    dv = DesignVars(d_body=0.09, lambda_body=7.0, S_fin=0.036, x_fin=0.50, AR_fin=2.2,
                    f_mount=0.8, n_design=4.0, d_prop=0.13, pd_prop=1.50, n_ser=6,
                    k_E=1.0, k_mot=1.0)
    pre = main.preprocess(dv)
    m_fixed = k.W_pl + sum(x[1] for x in k.AVIO_LIST) + k.N_rot * pre.pmap.m_prop

    def resp_of(M):
        st = strc.run(dv, pre.hull, pre.aero, M)
        return st.W_str, st

    return resp_of, m_fixed


def _report(rows):
    print(f"{'모델':<22} {'국소 S':>8} {'산포':>11} {'양자':>11} {'고유':>5} "
          f"{'창':>9} {'권고 floor_rel':>15}")
    print('-' * 88)
    for r in rows:
        q = '연속' if not r['staircase'] else f"{r['quantum']:.3e}"
        w = f"{r['rel_window']:.1e}" + ('*' if r['widened'] else ' ')
        # 계단형에서는 선형적합 기울기가 '계단 경계를 가로지른 할선'이라 국소 S 가 아니다.
        # 숫자를 그대로 찍으면 S 로 오해되므로 가린다.
        sl = '—' if r['staircase'] else f"{r['slope_local']:.4f}"
        print(f"{r['name']:<22} {sl:>8} {r['noise_abs']:>11.3e} "
              f"{q:>11} {r['n_unique']:>5} {w:>9} {r['floor_rel']:>15.3e}")
    if any(r['widened'] for r in rows):
        print("  * 창이 양자보다 좁아 자동으로 넓힌 행 (한 계단 안에 갇히면 양자를 못 잰다)")


if __name__ == '__main__':
    from common.out import stdout_utf8
    stdout_utf8()

    print("=" * 88)
    print("실측 — resid_floor / 양자화 크기")
    print("=" * 88)

    # (1) 실제 STRC — 지금 상태의 modules/strc.py 를 그대로 잰다
    resp_of, m_fixed = real_strc_case()
    from modules.wght import _iterate
    M_star = _iterate(k.k_init * m_fixed, m_fixed, resp_of)['MTOW']
    print(f"\n[1] 실제 modules/strc.py  (수렴점 MTOW = {M_star:.6f} kg)")
    _report([measure(resp_of, M_star, '실제 STRC (현재)')])
    print("  → 지금은 슬라이서 계수가 상수라 매끄럽다. k_sl_* 가 회귀 테이블로")
    print("    교체되면 계단형이 될 수 있으므로 그때 이 줄을 다시 재야 한다.")

    # (2) 스텁 — 도구가 양자를 정말 복원하는지 대조 (아는 답과의 비교)
    MTOW0 = 6.33        # 선형 스텁 S=0.21, m_fixed=5.0 의 수렴점 근방
    print(f"\n[2] 스텁 대조  (측정 중심 MTOW = {MTOW0} kg, 창 ±0.1%, 401점, 권고 = 3σ)")
    _report([
        measure(make_linear_strc(0.21), MTOW0, '선형 S=0.21'),
        measure(make_power_strc(0.35, 0.92), MTOW0, '멱함수 a=.35 b=.92'),
        measure(make_quantized_strc(0.21, 0.02), MTOW0, '양자화 q=0.02'),
        measure(make_quantized_strc(0.21, 0.002), MTOW0, '양자화 q=0.002'),
        measure(make_noisy_strc(make_linear_strc(0.21), 1e-9), MTOW0, '선형+노이즈 1e-9'),
        measure(make_noisy_strc(make_linear_strc(0.21), 1e-6), MTOW0, '선형+노이즈 1e-6'),
    ])
    print("  → 양자화 두 행이 넣은 양자(0.02·0.002)를 정확히 복원하면 도구가 맞는 것이다.")

    print("\n[읽는 법]")
    print("  · 매끄러운 모델은 노이즈≈0, 양자 '연속' → 하한 가드가 사실상 불필요.")
    print("    기본값 1e-6 은 이 경우 넉넉한 안전값이지 측정값이 아니다.")
    print("  · 계단형이면 노이즈가 아니라 양자가 floor 를 지배한다. 양자 미만의 resid")
    print("    차이는 물리적 의미가 없으므로 그 절반을 하한으로 권고한다.")
    print("  · 이보다 작게 잡으면 r̂ 이 노이즈만 재고, Ŝ 가 1 근처에도 이상에도 찍혀")
    print("    발산 오판이 난다. 그래서 하한 미달은 '이미 수렴'으로 보낸다.")
    print("\n[확정 절차]")
    print("  나온 floor_rel 을 constants.py 의 resid_floor 로 넣는다.")
