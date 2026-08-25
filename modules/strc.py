"""STRC — 프린트 구조 무게.  ICD0-008 §5.1

MTOW 에 반응하는 순수 구조 항이다. 이전 구조에서는 수렴 루프의 유일한 응답이었고,
지금은 셋(구조·모터·배터리) 중 하나다.

[구조 완료 — 슬라이서 계수 미확정]
기하 기반 질량 모형(쉘·핀·격벽)과 응력 검산은 구현했다. 다만 ICD §5.1 이 말한
"슬라이서 회귀"의 자리인 `k_sl_*` 가 **실측 대기**라 절대값은 보증되지 않는다.
반응 방향(n_design↑ → 무게↑, MTOW↑ → 무게↑)만 믿을 수 있다.

⚠ 슬라이서 출력은 레이어 수·둘레 수가 정수라 계단형일 수 있다. 그러면 MTOW 가 두 값
   사이를 진동하는데, 이건 발산이 아니라 limit_cycle 로 분류된다 (WGHT).
   지금 모형은 연속이라 그 현상이 안 나타난다 — 계수가 실측 테이블로 바뀌면 다시 본다.
"""
import math

import constants as k
from interfaces import DesignVars, HullOut, AeroOut, StrcOut, MassItem


def infill(n_design: float) -> float:
    """인필율 — 하중배수가 높을수록 채운다 (§5.1 "인필율·벽두께·둘레 수")."""
    return min(max(k.phi_0 + k.k_phi * (n_design - k.n_ref_load), 0.0), 1.0)


def wall_thickness(n_design: float) -> float:
    """벽두께 [m]. GEOM 이 내부 지름을 낼 때 쓰는 것과 **같은 규칙**이다.

    ⚠ 원래 이 규칙의 주인은 STRC 다 (§5.1 "인필율·벽두께·둘레 수").
      GEOM 이 배치를 위해 같은 식을 복제해 두고 있다 — docs §11-27.
      한쪽만 고치면 조용히 어긋나므로 값을 바꿀 때 둘 다 본다. [확정 필요]
    """
    return k.t_0 + k.k_t * (n_design - k.n_ref_load)


def _t_for_stress(M: float, d_body: float) -> float:
    """굽힘 모멘트 M 을 허용응력 안에서 받는 최소 벽두께 [m] — 결정론적 이분법.

    중공 원통 단면계수 Z = π(r_o⁴ − r_i⁴)/(4 r_o), σ = M/Z ≤ σ_allow/SF.
    """
    sig = k.sigma_allow / k.SF
    r_o = 0.5 * d_body
    lo, hi = 1e-7, r_o * 0.9
    for _ in range(k.N_bisect_max):
        t = 0.5 * (lo + hi)
        r_i = max(r_o - t, 1e-9)
        Z = math.pi * (r_o ** 4 - r_i ** 4) / (4.0 * r_o)
        if M / max(Z, 1e-18) > sig:
            lo = t
        else:
            hi = t
        if hi - lo < 1e-9:
            break
    return hi


def run(dv: DesignVars, hl: HullOut, aer: AeroOut, MTOW: float) -> StrcOut:
    """① 프린트 구조 무게 — 설계 하중에서 구조를 낸다.

    벽두께·인필은 **두 요구의 최대값**이다:
      · 프린트 하한 : t_0 + k_t·(n_design − n_ref).  얇으면 못 뽑는다
      · 응력 요구   : 설계 하중 n_design·MTOW·g + 공력 하중을 허용응력 안에서 받는 두께

    ICD §5.1 은 "슬라이서 회귀: 인필율·벽두께 → 무게" 와 "응력 검산으로 g5" 를 적어
    두께를 **입력**으로 읽히는데, 그러면 W_str 이 MTOW 에 전혀 반응하지 않아
    §10 의 "응답 질량 = MTOW 에 따라 변하는 질량 (구조·모터·배터리)" 와 어긋난다.
    최대값을 취하면 둘 다 만족한다 — 가벼우면 하한이, 무거우면 응력이 정한다.
    [로컬 개정 — docs §11-33]

    ⚠ 실측: 이 설계공간에서는 **하한이 6~88배로 압도적**이다 (MTOW 0.7 kg 에서
      응력 요구 0.02 mm vs 하한 1.80 mm). 즉 구조는 응력이 아니라 프린트 가능성이
      정하고, 결과적으로 W_str 은 MTOW 에 거의 반응하지 않는다. 무게 스노우볼은
      모터·배터리가 만든다.
    """
    # ── 설계 하중 ──
    W_load = dv.n_design * MTOW * k.g                    # 관성 하중 [N]
    N_fin = aer.q_cr * hl.S_ref * aer.CN_alpha_fin * k.alpha_lim   # 핀 법선력 4매 합 [N]
    M_body = W_load * 0.25 * hl.l_body                   # 최대 단면 굽힘 [N·m]

    # ── 벽두께 — 프린트 하한과 응력 요구의 최대값 ──
    t_floor = wall_thickness(dv.n_design)
    t_stress = _t_for_stress(M_body, dv.d_body)
    t_w = max(t_floor, t_stress)
    sized_by = "print" if t_floor >= t_stress else "stress"

    # ── 인필 — 핀 뿌리 굽힘이 요구하는 최소치와 하한의 최대값 ──
    phi_floor = infill(dv.n_design)
    M_fin = (N_fin / k.N_rot) * (0.5 * hl.b_fin)         # 1매, 스팬 중심 하중 (보수적)
    Z_fin = hl.c_root * hl.t_fin ** 2 / 6.0
    phi_stress = M_fin / max(Z_fin * k.sigma_allow / k.SF, 1e-12)
    phi = min(max(phi_floor, phi_stress), 1.0)

    # ── 질량 ──
    m_shell = hl.S_wet_body * t_w * k.rho_mat * k.k_sl_shell
    m_fin = dv.S_fin * hl.t_fin * phi * k.rho_mat * k.k_sl_fin
    A_sec = math.pi * (dv.d_body - 2.0 * t_w) ** 2 / 4.0
    m_bulk = k.N_bulk * A_sec * k.t_bulk * k.rho_mat * k.k_sl_bulk
    W_str = m_shell + m_fin + m_bulk

    # ── g5 — 사이징된 두께·인필에서의 실제 응력 여유 ──
    r_o = 0.5 * dv.d_body
    r_i = max(r_o - t_w, 1e-9)
    Z_body = math.pi * (r_o ** 4 - r_i ** 4) / (4.0 * r_o)
    sig_allow = k.sigma_allow / k.SF
    sig_body = M_body / max(Z_body, 1e-12)
    sig_fin = M_fin / max(Z_fin * phi, 1e-12)
    g5 = 1.0 - max(sig_fin, sig_body) / sig_allow        # 양수 합격 (여유비)

    # 쉘 질량은 **젖음면적 도심**에 놓는다. 원통 중점에 두면 노즈분(전체의 36%)이
    # 통째로 뒤로 밀려 x_cg 가 4 cm 가까이 틀어지고 SM 이 0.45 cal 낮게 나온다.
    x_shell = hl.x_wet_body
    bd = [
        MassItem("shell", m_shell, x_shell, hl.r_body),
        MassItem("fins", m_fin, hl.x_fin + 0.5 * hl.c_root, 0.5 * hl.b_fin),
        MassItem("bulkheads", m_bulk, x_shell, 0.0),
    ]
    st = StrcOut(W_str=W_str, m_print=W_str, g5=g5, breakdown_str=bd)
    st.t_wall = t_w          # GEOM 이 내부 지름에 쓴다 (docs §11-27)
    st.sized_by = sized_by   # 진단: 두께를 정한 것이 프린트 하한인가 응력인가
    return st


if __name__ == "__main__":   # 검산 — 반응 방향과 크기가 말이 되는가
    from common.out import stdout_utf8
    from modules import atm, geom, aero
    stdout_utf8()

    dv = DesignVars(d_body=0.09, lambda_body=7.0, S_fin=0.036, x_fin=0.50,
                    AR_fin=2.2, f_mount=0.8, n_design=4.0, d_prop=0.13,
                    pd_prop=1.50, n_ser=6, k_E=1.0, k_mot=1.0)
    air = atm.run(0.0); hl = geom.hull(dv); aer = aero.run(dv, hl, air)

    import dataclasses
    print(f"{'n_design':>9} {'t_wall[mm]':>11} {'인필':>6} {'W_str[g]':>9} {'g5':>8}")
    prev = None
    for n in (4.0, 6.0, 8.0, 10.0):
        st = run(dataclasses.replace(dv, n_design=n), hl, aer, 0.68)
        print(f"{n:>9.1f} {wall_thickness(n)*1e3:>11.3f} {infill(n):>6.2f} "
              f"{st.W_str*1e3:>9.2f} {st.g5:>+8.4f}")
        if prev is not None:
            assert st.W_str > prev, "n_design 을 올렸는데 구조가 가벼워진다"
        prev = st.W_str

    print()
    print(f"{'MTOW[kg]':>9} {'t_wall[mm]':>11} {'정한 것':>8} {'W_str[g]':>9} {'g5':>8}")
    prev = None
    for M in (0.5, 1.5, 5.0, 20.0, 60.0):
        st = run(dv, hl, aer, M)
        print(f"{M:>9.1f} {st.t_wall*1e3:>11.3f} {st.sized_by:>8} {st.W_str*1e3:>9.2f} {st.g5:>+8.4f}")
        if prev is not None:
            assert st.W_str >= prev - 1e-12, "MTOW 를 늘렸는데 구조가 가벼워진다"
        prev = st.W_str
    print("  → 가벼울 때는 프린트 하한이, 무거워지면 응력이 두께를 정한다.")
    print("    이 설계공간(MTOW ~1 kg)에서는 하한이 압도적이라 W_str 이 MTOW 에 거의")
    print("    반응하지 않는다 — 무게 스노우볼은 모터·배터리가 만든다. (docs §11-33)")
    assert run(dv, hl, aer, 60.0).W_str > run(dv, hl, aer, 0.5).W_str, "응력 구간에서 반응해야 한다"
    assert run(dv, hl, aer, 0.5).g5 > 0, "가벼운데 응력 불합격"
    print("STRC 검산 통과")
