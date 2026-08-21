"""STRC — 구조 무게. [골격 구현, 슬라이서 계수는 스텁]
가이드라인: 「STRC 계산 가이드라인 — 구조 무게」
구조식은 가이드라인 §3~§7대로. k_sl_* 슬라이서 회귀 계수만 실측으로 교체.
MTOW에 반응하는 항은 핀 속 채움(m_fill) 하나뿐 — 이 분리가 루프를 지킨다.
"""
import constants as k
from interfaces import DesignVars, GeomOut, StrcOut, MassItem


def fixed_masses(dv: DesignVars, geo: GeomOut):
    """MTOW 무관 항 — 루프 진입 전 1회 계산 (STRC §7)."""
    phi = k.phi_0 + k.k_phi * (dv.n_design - k.n_ref_load)
    t_skin = k.n_peri * k.w_line
    m_shell = k.rho_mat * (geo.S_wet["nose"] + geo.S_wet["cyl"]) * geo.t_wall * k.k_sl_shell
    S_1 = dv.S_fin / k.N_rot
    V_fin1 = k.k_sec * S_1 * geo.t_fin
    m_fin1 = k.rho_mat * (2.0 * S_1 * t_skin
                          + phi * max(V_fin1 - 2.0 * S_1 * t_skin, 0.0)) * k.k_sl_fin
    m_pod1 = k.rho_mat * 3.1416 * geo.d_pod * geo.l_pod * k.t_wall_pod * k.k_sl_pod
    m_bulk = k.N_bulk * k.rho_mat * 3.1416 * (geo.d_int / 2) ** 2 * 0.002 * k.k_sl_inf
    m_reinf = k.k_reinf * (k.N_rot * m_fin1 + k.N_rot * m_pod1)
    parts = {"shell": m_shell, "fin": k.N_rot * m_fin1,
             "pod_shell": k.N_rot * m_pod1, "bulk": m_bulk, "reinf": m_reinf}
    return parts


def run(dv: DesignVars, geo: GeomOut, q_cr, CN_alpha_fin, MTOW, fixed=None) -> StrcOut:
    if fixed is None:
        fixed = fixed_masses(dv, geo)

    # §5 설계 하중 (MTOW 의존)
    T_lim = dv.n_design * MTOW * k.g / k.N_rot
    S_1 = dv.S_fin / k.N_rot
    N_aero = q_cr * S_1 * (CN_alpha_fin / k.N_rot) * k.alpha_lim
    M_root = T_lim * geo.root_arm + N_aero * (k.k_ycp * geo.b_1)

    # §6 핀 속 채움 사이징
    sigma_allow = k.k_layer * k.sigma_cat / k.SF
    w_fill = 6.0 * M_root / (sigma_allow * geo.t_fin ** 2)
    m_fill = (k.rho_mat * k.k_dens * w_fill * geo.t_fin * geo.b_1
              * k.k_taper * k.N_rot)

    m_str = sum(fixed.values()) + m_fill
    g10 = k.k_w * geo.c_r / max(w_fill, 1e-9) - 1.0

    # breakdown — {name, m, x, r} (WGHT의 x_cg·J용)
    x_shell = geo.l_nose * 0.6 + geo.l_cyl / 2.0     # 근사 도심
    r_fin = geo.r_body + k.k_r * geo.b_1
    bd = [
        MassItem("shell", fixed["shell"], x_shell, geo.r_body),
        MassItem("fin", fixed["fin"] + m_fill, geo.x_fin + geo.c_r / 2.0, r_fin),
        MassItem("pod_shell", fixed["pod_shell"], geo.x_pod, geo.arm_rotor),
        MassItem("bulk", fixed["bulk"], geo.l_nose + geo.l_cyl / 2.0, 0.0),
        MassItem("reinf", fixed["reinf"], geo.x_fin + geo.c_r / 2.0, geo.r_body),
    ]
    return StrcOut(m_str=m_str, m_print=m_str, m_fill=m_fill,
                   w_fill=w_fill, breakdown_str=bd, g10=g10)
