"""WGHT 검증 — 수렴 루프가 이론대로 도는가

실행: python3 -m validation.wght.test_wght   (저장소 루트에서)

[검증 전략]
WGHT 는 C-2 에서 수천 후보에 반복 사용될 코드다. 그런데 실제 응답 질량(구조·모터·
배터리)은 계수가 실측 대기라 '정답'을 모르므로, 정답을 아는 가짜 응답 모델(선형 스텁)과
대조한다.

선형 스텁 W_str = S·MTOW 는 셋을 해석적으로 안다:
  · 고정점    MTOW* = m_fixed/(1-S)
  · 발산 경계 정확히 S = 1 (beta 와 무관)
  · Ŝ 는 beta 를 어떻게 잡든 S 를 복원해야 한다

숫자가 이론과 다르면 "경계가 그 값이다"가 아니라 "우리 코드에 버그가 있다"가 정답이다.

테스트 1~12·14·15 는 _iterate() 를 직접 겨눈다 (스텁 주입).
테스트 3·13·16 은 실제 파이프라인 main.evaluate() 를 겨눈다 (질량 특성·항등식).

[ICD0-008 이식 — 바뀐 것은 배선뿐이다]
  · 주입 함수 : strc_of(MTOW) -> StrcOut  →  resp_of(MTOW) -> (응답질량_합, payload)
  · 전처리    : 이 파일의 pipeline() 헬퍼를 없애고 main.preprocess() 를 부른다 (§8 C-1)
  · 질량 특성 : converge() 가 아니라 wght.mass_props() 가 낸다 (배치가 ②라서)
assert 내용은 하나도 바뀌지 않았다.
"""
import math
import sys

import constants as k
import main
from interfaces import DesignVars
from modules import strc
from modules.wght import _iterate
from validation.wght.strc_stub import (make_linear_strc, make_power_strc,
                                       make_quantized_strc, make_impure_strc)

M_FIXED = 5.0          # 임의의 고정 질량 [kg] — 값 자체는 결과에 영향이 없다

# 실제 파이프라인용 대표 설계점 — 범위가 전부 TBD 라 이 숫자들도 스모크용이다.
BASE_DV = dict(d_body=0.09, lambda_body=8.0, S_fin=0.036, x_fin=0.55, AR_fin=2.2,
               f_mount=1.0, n_design=4.0, d_prop=0.13, pd_prop=1.50, n_ser=6,
               k_E=1.0, k_mot=1.0)


def run(resp_of, **opts):
    """스텁을 _iterate 에 물린다. 초기값은 실제 converge() 와 같은 규칙."""
    return _iterate(k.k_init * M_FIXED, M_FIXED, resp_of, opts or None)


def sizing(**over):
    """실제 파이프라인 한 바퀴 — ⓪ 전처리는 main 이 단일 출처다 (§8 C-1)."""
    dv = DesignVars(**{**BASE_DV, **over})
    return dv, main.evaluate(dv)


def check(cond, msg):
    if not cond:
        print(f"  FAIL: {msg}")
        raise AssertionError(msg)


def banner(n, title):
    print(f"\n{'=' * 70}\nTEST {n}: {title}")


# ─────────────────────────────────────────────────────────────────────────────
def test_1_analytic():
    banner(1, "선형 스텁 — 해석해 대조 (고정점·Ŝ)")
    S = 0.21
    r = run(make_linear_strc(S))
    exact = M_FIXED / (1 - S)
    rel = abs(r['MTOW'] - exact) / exact

    print(f"  status={r['status']}  n_iter={r['n_iter']}  Ŝ={r['S_hat']:.9f}")
    print(f"  MTOW   = {r['MTOW']:.9f}")
    print(f"  해석해 = {exact:.9f}   상대오차 = {rel:.3e}")

    check(r['status'] == 'converged', "수렴해야 한다")
    check(abs(r['S_hat'] - S) < 1e-9, f"Ŝ 가 S 를 복원해야 한다: {r['S_hat']} vs {S}")
    # 실제 정확도의 상한은 eps 가 아니라 eps·Ŝ 다 (판정용 오차와 반환값 오차가 다르다)
    check(rel <= k.eps_conv * S,
          f"실제 오차가 상한 eps·S 이내여야 한다: {rel:.3e} > {k.eps_conv * S:.3e}")
    print("  PASS")


def test_2_beta_invariance():
    banner(2, "beta 불변성 — 완화 격리 원칙")
    S = 0.21
    rows = []
    for beta in [0.3, 0.5, 1.0, 1.5, 2.0]:
        r = run(make_linear_strc(S), beta=beta, eps_conv=1e-5)
        rows.append((beta, r['MTOW'], r['S_hat'], r['n_iter']))
        print(f"  beta={beta:4.1f}  MTOW={r['MTOW']:.9f}  Ŝ={r['S_hat']:.9f}  n={r['n_iter']}")

    # Ŝ 가 beta 와 무관해야 한다 — 이게 'beta 불변 지표'라고 부른 근거다
    for beta, _, sh, _ in rows:
        check(abs(sh - S) < 1e-9, f"beta={beta} 에서 Ŝ 가 어긋남: {sh}")
    spread = max(m for _, m, _, _ in rows) - min(m for _, m, _, _ in rows)
    print(f"  MTOW 최대편차 = {spread:.3e}")
    check(spread / rows[0][1] < 1e-5, f"MTOW 가 beta 에 끌려다님: 편차 {spread:.3e}")
    print("  PASS")


def test_3_sum_identity():
    banner(3, "항등식 — MTOW == Σbreakdown (실제 파이프라인)")
    dv, res = sizing()
    bd = res.mass.breakdown
    total = sum(i.m for i in bd)
    d = abs(total - res.wght.MTOW)
    print(f"  MTOW={res.wght.MTOW:.12f}  Σbreakdown={total:.12f}")
    print(f"  |차이| = {d:.3e}   항목 {len(bd)}개")
    # 부동소수 합산 순서 차이만 허용. 항등식이므로 이 이상 벌어지면 안 된다.
    check(d <= 1e-12 * max(1.0, res.wght.MTOW), f"항등식이 깨짐 ({d:.3e})")
    print("  PASS")


def test_4_structural_divergence():
    banner(4, "구조 발산 — 경계가 정확히 S=1")
    print(f"  {'S':>6} {'status':>22} {'Ŝ':>9} {'err':>10}")
    for S, want in [(0.99, 'max_iter'), (1.00, 'diverged_structural'),
                    (1.10, 'diverged_structural'), (2.00, 'diverged_structural')]:
        r = run(make_linear_strc(S))
        er = 'None' if r['err'] is None else f"{r['err']:.2e}"
        print(f"  {S:>6.2f} {r['status']:>22} {r['S_hat']:>9.4f} {er:>10}")
        check(r['status'] == want, f"S={S}: {want} 여야 하는데 {r['status']}")
        if want.startswith('diverged'):
            check(r['err'] is None, f"S={S}: Ŝ>=1 이면 err 은 None 이어야 한다")
            check(abs(r['S_hat'] - S) < 1e-9, f"S={S}: Ŝ 가 S 를 복원해야 한다")
    print("  PASS")


def test_5_hole_regression():
    banner(5, "[회귀] Ŝ ∈ (1, 1+delta_r) 구멍 — 발산을 수렴으로 뒤집던 자리")
    # deadband 를 두면 이 구간이 어느 status 에도 안 걸리고, 수렴 판정의 분모
    # (1-Ŝ) 가 음수가 되어 판정식 (< eps·MTOW) 을 무조건 통과한다.
    #   Ŝ=1.02 → |resid|/(1-1.02) < 0 < eps·MTOW  → 항상 참
    # 그래서 (a) r̂>=1 을 deadband 없이 즉시 발산으로 잡고 (b) 판정식에도 Ŝ<1 을 명시했다.
    for S in [1.001, 1.02, 1.049]:
        r = run(make_linear_strc(S))
        print(f"  S={S:<6} status={r['status']:22s} Ŝ={r['S_hat']:.4f}  err={r['err']}")
        check(r['status'] == 'diverged_structural', f"S={S} 는 발산이어야 한다 (구멍 재발)")
        check(r['err'] is None, f"S={S}: err 이 None 이어야 한다 (음수 유출 금지)")
    # 음수 err 이 어떤 경로로도 새어나가지 않는지 전 구간 확인
    for S in [0.2, 0.5, 0.9, 0.99, 1.0, 1.02, 1.5, 2.0]:
        r = run(make_linear_strc(S))
        check(r['err'] is None or r['err'] >= 0, f"S={S}: 음수 err 유출 ({r['err']})")
    print("  음수 err 유출 없음 (S=0.2~2.0 전 구간)")
    print("  PASS")


def test_6_numerical_divergence():
    banner(6, "수치 발산 — beta > 2/(1-S) 에서만, 그리고 구조 발산과 구분된다")
    S = 0.21
    limit = 2 / (1 - S)
    print(f"  이론 안정한계 beta < 2/(1-S) = {limit:.4f}")
    print(f"  {'beta':>6} {'h_theory':>9} {'status':>22}")
    for beta in [0.5, 1.0, 2.0, 2.4, 2.6, 3.0]:
        h = 1 + beta * (S - 1)
        r = run(make_linear_strc(S), beta=beta)
        print(f"  {beta:>6.2f} {h:>9.4f} {r['status']:>22}")
        if abs(h) < 0.99:                       # 여유를 두고 안정 구간만 단정
            check(r['status'] == 'converged', f"beta={beta}: 안정 구간인데 {r['status']}")
        if abs(h) > 1.01:                       # 확실한 불안정 구간
            check(r['status'] == 'diverged_numerical',
                  f"beta={beta}: S<1 이므로 numerical 이어야 한다 "
                  f"(구조 발산과 같은 색이면 누가 고쳐야 하는지 알 수 없다) — {r['status']}")
    print("  PASS")


def test_7_limit_cycle():
    banner(7, "리밋 사이클 — 중점이 아니라 '분기를 통째로' 채택")
    found = False
    for q, beta in [(0.02, 2.0), (0.02, 2.5), (0.05, 2.2), (0.1, 2.4)]:
        r = run(make_quantized_strc(0.21, q), beta=beta)
        if r['status'] != 'limit_cycle':
            continue
        found = True
        print(f"  quantum={q} beta={beta}: status={r['status']}  n_iter={r['n_iter']}  "
              f"err={r['err']:.4e}")

        # 핵심: 중점을 반환하면 어떤 응답 모델 호출 결과와도 같지 않아 합성값이 된다.
        # 반환값이 실제 이력에 있던 반복의 raw 중 하나인지로 확인한다.
        raws = [h[1] for h in r['history']]
        check(any(abs(r['MTOW'] - x) <= 1e-12 * r['MTOW'] for x in raws),
              f"q={q}: 반환 MTOW 가 어떤 반복의 raw 와도 일치하지 않는다 (합성값)")

        # err 은 두 분기 반환값 차의 절반이어야 한다
        gap = abs(raws[-1] - raws[-2])
        print(f"     두 분기 raw 차={gap:.4e}  err={r['err']:.4e}  (기대 {gap / 2:.4e})")
        check(r['err'] is not None and r['err'] >= 0, f"q={q}: err 이 음수/None")
        check(abs(r['err'] - gap / 2) <= 1e-12 * max(1.0, gap), f"q={q}: err != 양자/2")

        # 반환된 payload 가 채택한 분기의 것인지 — W_str 이 raw 와 정합해야 한다
        check(abs((M_FIXED + r['payload'].W_str) - r['MTOW']) <= 1e-12 * r['MTOW'],
              f"q={q}: 반환 payload 가 채택 분기의 것이 아니다")
    check(found, "리밋 사이클을 한 번도 재현하지 못했다 — 스윕 격자를 넓혀야 한다")
    print("  → err=0 이 나와도 버그가 아니다: 두 분기가 같은 raw 를 주면 그 값이 정확한 고정점이다")
    print("  PASS")


def test_8_quantization_at_ship_setting():
    banner(8, "출하 설정(beta=0.5)에서는 양자화가 사이클을 만들지 않는다")
    # 0 < beta <= 1 이면 반복식이 감소 사상이 될 수 없다. beta=1 이면 M_{k+1}=g(M_k) 이고
    # g 가 단조증가면 2-주기 궤도가 없다 (a<b, f(a)=b>a, f(b)=a<b 이면 f(a)>f(b) 로 모순).
    # beta<1 이면 더욱 그렇다. 계단형이어도 마찬가지다.
    print(f"  {'quantum':>9} {'status':>12} {'n_iter':>7}")
    for q in [0.005, 0.02, 0.1, 0.5, 2.0]:
        r = run(make_quantized_strc(0.21, q), beta=k.beta)
        print(f"  {q:>9.3f} {r['status']:>12} {r['n_iter']:>7}")
        check(r['status'] == 'converged',
              f"q={q}: beta<=1 에서는 수렴해야 한다 (사이클은 beta>1 에서만) — {r['status']}")
    print("  → limit_cycle 기계는 출하 설정용이 아니라 보험이다.")
    print("    다만 W_str 이 국소적으로 비단조인 회귀식이면 beta<=1 에서도 가능하므로 유지한다")
    print("  PASS")


def test_9_min_iter_rule():
    banner(9, "r̂ 추정 전 수렴 선언 금지")
    for S in [0.05, 0.21, 0.4, 0.6]:
        r = run(make_linear_strc(S))
        floor_path = r['S_hat'] is None         # 하한 가드 경로는 예외
        print(f"  S={S:<5} status={r['status']:10s} n_iter={r['n_iter']}  "
              f"{'(하한 가드)' if floor_path else ''}")
        if r['status'] == 'converged' and not floor_path:
            check(r['n_iter'] >= k.n_min_iter,
                  f"S={S}: {r['n_iter']}회 만에 수렴 선언 — r̂ 없이 판정했다")
    print("  PASS")


def test_10_max_iter_reports_err():
    banner(10, "max_iter 는 err 을 동반한다 — 이진 판단을 호출부에 넘기지 않는다")
    r = run(make_linear_strc(0.95))
    print(f"  S=0.95  status={r['status']}  n_iter={r['n_iter']}  "
          f"Ŝ={r['S_hat']:.4f}  err={r['err']:.4e}")
    check(r['status'] == 'max_iter', f"max_iter 여야 하는데 {r['status']}")
    check(r['err'] is not None and r['err'] > 0,
          "err 을 같이 줘야 호출부가 임계로 처리할 수 있다")
    print("  PASS")


def test_11_purity():
    banner(11, "주입 함수의 무상태성 — 이게 깨지면 배치에서 설계점이 서로 오염된다")
    # (a) 실제 modules.strc.run 이 순수한가 — 같은 MTOW 두 번이 같은 답을 줘야 한다
    dv = DesignVars(**BASE_DV)
    pre = main.preprocess(dv)
    m1 = strc.run(dv, pre.hull, pre.aero, 1.5).W_str
    m2 = strc.run(dv, pre.hull, pre.aero, 1.5).W_str
    print(f"  실제 STRC: W_str(1.5) 두 번 → {m1:.12f} / {m2:.12f}")
    check(m1 == m2, "modules.strc.run 이 상태를 들고 있다 — 배치에서 오염된다")

    # (b) 상태를 든 스텁은 결과를 바꿔야 한다 (테스트 자체가 유효한지 확인)
    impure = make_impure_strc(0.21)
    a1 = run(impure)['MTOW']
    a2 = run(impure)['MTOW']
    print(f"  비순수 스텁: 1회차 {a1:.9f} / 2회차 {a2:.9f}  → 다름={a1 != a2}")
    check(a1 != a2, "비순수 스텁인데 결과가 같다 — 이 테스트가 아무것도 검증하지 못한다")
    print("  PASS")


def test_12_exceptions():
    banner(12, "설정 함정은 조용히 넘기지 않고 예외로 막는다")
    # (1) eps_conv <= resid_floor 면 하한 가드가 판정을 항상 선점한다.
    #     status 는 converged 로 나오지만 Ŝ=None 이라 오차 보고가 불가능해진다.
    try:
        run(make_linear_strc(0.21), eps_conv=1e-8)
        check(False, "eps<=resid_floor 를 통과시켰다")
    except ValueError as e:
        print(f"  eps<=resid_floor → ValueError ✓  ({str(e)[:46]}...)")
    # (2) N_iter_max < 1 이면 루프가 한 번도 안 돌아 반환값이 정의되지 않는다
    try:
        run(make_linear_strc(0.21), N_iter_max=0)
        check(False, "N_iter_max=0 을 통과시켰다")
    except ValueError:
        print("  N_iter_max<1     → ValueError ✓")
    print("  PASS")


def test_13_mass_properties():
    banner(13, "질량 특성 — 3축 관성이 배치를 반영하는가")
    dv, res = sizing(f_mount=0.4)
    mp = res.mass

    # (a) x_cg 는 breakdown 의 가중평균이어야 한다 (다른 경로로 계산하고 있지 않은지)
    m_tot = sum(i.m for i in mp.breakdown)
    x_ref = sum(i.m * i.x for i in mp.breakdown) / m_tot
    print(f"  (a) x_cg={mp.x_cg:.9f}  가중평균={x_ref:.9f}")
    check(abs(mp.x_cg - x_ref) < 1e-12, "x_cg 가 breakdown 의 가중평균이 아니다")

    # (b) 축대칭 전제 → J_yy == J_zz
    print(f"  (b) J_yy={mp.J_yy * 1e3:.4f} g·m²  J_zz={mp.J_zz * 1e3:.4f} g·m²")
    check(mp.J_yy == mp.J_zz, "축대칭 전제인데 J_yy != J_zz")

    # (c) 로터 암을 늘리면 J_xx 가 커져야 한다. 안 변하면 반경 r 을 안 쓰는 것이고,
    #     그러면 롤 관성이 계통적으로 과소평가된다.
    #     arm_rotor 는 f_mount(포드의 스팬 방향 결합 위치)의 함수이므로
    #     루트쪽 0.4 에서 팁쪽 1.0 으로 옮겨 암을 늘린다.
    dv2, res2 = sizing(f_mount=1.0)
    print(f"  (c) arm_rotor {res.layout.arm_rotor:.4f} → {res2.layout.arm_rotor:.4f} m  "
          f"J_xx {mp.J_xx * 1e3:.3f} → {res2.mass.J_xx * 1e3:.3f} g·m²")
    check(res2.layout.arm_rotor > res.layout.arm_rotor,
          "테스트 전제 불성립: arm_rotor 가 안 커졌다")
    check(res2.mass.J_xx > mp.J_xx, "arm_rotor 를 늘렸는데 J_xx 가 안 커진다 — 반경 r 미반영")

    # (d) 모든 관성은 양수여야 한다
    check(mp.J_xx > 0 and mp.J_yy > 0 and mp.J_zz > 0, "관성이 0 이하")
    print("  PASS")


def test_14_output_shape():
    banner(14, "출력 규격 — history 3튜플·resid 부호·반환 키")
    r = run(make_linear_strc(0.21))
    h = r['history']
    print(f"  history 길이={len(h)}  첫 항목={tuple(round(v, 6) for v in h[0])}")
    check(all(len(t) == 3 for t in h), "history 는 (MTOW_k, raw, resid_k) 3튜플이어야 한다")
    check(len(h) == r['n_iter'], "history 길이가 n_iter 와 다르다")

    # 부호가 없으면 과완화 구간(h<0) 사후 분석이 불가능하다
    r_over = run(make_linear_strc(0.21), beta=2.0, eps_conv=1e-5)
    signs = {math.copysign(1, t[2]) for t in r_over['history']}
    print(f"  beta=2.0 의 resid 부호 집합 = {signs}")
    check(len(signs) > 1, "과완화인데 resid 부호가 한 종류 — 절댓값으로 저장하고 있다")

    for key in ['MTOW', 'payload', 'status', 'S_hat', 'err', 'n_iter', 'history']:
        check(key in r, f"출력 키 누락: {key}")
    print(f"  반환 키 7종 전부 존재")

    # 반환 MTOW 는 '완화된 반복값'이 아니라 마지막 응답 모델 호출의 합이어야 한다
    check(abs((M_FIXED + r['payload'].W_str) - r['MTOW']) <= 1e-12 * r['MTOW'],
          "반환 MTOW 가 마지막 응답 모델 호출과 정합하지 않는다")
    print("  PASS")


def test_15_statelessness():
    banner(15, "_iterate 자신의 무상태성 — 같은 입력이면 같은 출력")
    fn = make_linear_strc(0.21)
    a = run(fn)
    b = run(fn)
    c = run(make_linear_strc(1.5))      # 사이에 발산 설계점을 끼워넣는다
    d = run(fn)
    print(f"  1회차 MTOW={a['MTOW']:.12f}")
    print(f"  2회차 MTOW={b['MTOW']:.12f}")
    print(f"  발산 설계점(S=1.5) 통과 후 MTOW={d['MTOW']:.12f}  (status={c['status']})")
    check(a['MTOW'] == b['MTOW'] == d['MTOW'],
          "호출 순서가 결과를 바꾼다 — 상태가 남아 있다")
    print("  PASS")


def test_16_avio_list_coverage():
    banner(16, "[회귀] 항전 목록이 늘어도 질량이 사라지지 않는다")
    # breakdown 의 항전 분류가 품목명을 하드코딩하면, 목록에 품목이 늘 때
    # m_fixed 에는 반영되는데 breakdown 에는 안 들어가고, 반환 MTOW 가 곧
    # Σbreakdown 이므로 그 차액이 예외도 경고도 없이 사라진다.
    orig = k.AVIO_LIST
    try:
        for label, extra in [
            ("기존 4종", []),
            ("RTK 로버 추가", [("rtk_rover", 0.030, 0.05, 0.03, 0.01, 300000.0)]),
            ("RTK + BEC 추가", [("rtk_rover", 0.030, 0.05, 0.03, 0.01, 300000.0),
                                ("bec", 0.015, 0.03, 0.02, 0.01, 20000.0)]),
        ]:
            k.AVIO_LIST = orig + extra
            dv, res = sizing()
            pl = res.wght.payload
            want = (k.W_pl + sum(x[1] for x in k.AVIO_LIST)
                    + k.N_rot * res.pre.pmap.m_prop
                    + k.N_rot * pl.m_mot + pl.m_batt + pl.m_pack + pl.W_str)
            d = res.wght.MTOW - want
            print(f"  {label:16s} MTOW={res.wght.MTOW:.9f}  기대={want:.9f}  "
                  f"차이={d * 1000:+.4f} g")
            check(abs(d) <= 1e-9 * max(1.0, want),
                  f"{label}: 질량 {d * 1000:+.3f} g 가 사라졌다")
    finally:
        k.AVIO_LIST = orig
    print("  PASS")


ALL = [test_1_analytic, test_2_beta_invariance, test_3_sum_identity,
       test_4_structural_divergence, test_5_hole_regression, test_6_numerical_divergence,
       test_7_limit_cycle, test_8_quantization_at_ship_setting, test_9_min_iter_rule,
       test_10_max_iter_reports_err, test_11_purity, test_12_exceptions,
       test_13_mass_properties, test_14_output_shape, test_15_statelessness,
       test_16_avio_list_coverage]

if __name__ == '__main__':
    from common.out import stdout_utf8
    stdout_utf8()
    failed = []
    for t in ALL:
        try:
            t()
        except AssertionError as e:
            failed.append((t.__name__, str(e)))
    print(f"\n{'=' * 70}")
    if failed:
        print(f"FAILED {len(failed)}/{len(ALL)}")
        for n, m in failed:
            print(f"  - {n}: {m}")
        sys.exit(1)
    print(f"ALL {len(ALL)} TESTS PASSED")
    print('=' * 70)
