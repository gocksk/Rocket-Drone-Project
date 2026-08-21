"""WGHT — 무게 합산·수렴. [실제 구현 — 통합의 핵심]
가이드라인: 「WGHT 계산 가이드라인 — 무게 합산·수렴」
DSM ②구획 {STRC ↔ WGHT} 루프가 여기다. 피드백은 MTOW 단 하나.
"""
import constants as k
from interfaces import DesignVars, GeomOut, WghtOut, MassItem
from common import srl
from modules import strc


def converge(dv: DesignVars, geo: GeomOut, aero, m_propsys) -> WghtOut:
    # §2 고정 질량 (루프 진입 전 1회)
    m_batt = dv.E_batt / k.e_spec
    m_pack = k.k_pack * m_batt
    m_avio = sum(a[1] for a in srl.avio())
    m_fixed = m_propsys + m_batt + m_pack + m_avio
    strc_fixed = strc.fixed_masses(dv, geo)

    # §3 수렴 루프
    MTOW = k.k_init * m_fixed
    st, converged, it = None, False, 0
    for it in range(1, k.N_iter_max + 1):
        st = strc.run(dv, geo, aero.q_cr, aero.CN_alpha_fin, MTOW, strc_fixed)
        MTOW_new = m_fixed + st.m_str
        if abs(MTOW_new - MTOW) / MTOW < k.eps_conv:
            MTOW, converged = MTOW_new, True
            break
        MTOW = MTOW + k.beta * (MTOW_new - MTOW)   # 완화

    # §4 무게중심 — 모든 위치는 기수 기준
    bd = list(st.breakdown_str)
    bd += [
        MassItem("motors_props", k.N_rot * (m_propsys / (1 + k.k_wire)) / k.N_rot * 0
                 + m_propsys * 0.85, geo.x_pod, geo.arm_rotor),      # 모터·프롭·ESC 대부분 포드에
        MassItem("wires", m_propsys * 0.15, geo.l_nose + geo.l_cyl / 2, geo.r_body * 0.5),
        MassItem("batt", m_batt + m_pack, geo.x_parts["batt"], 0.0),
        MassItem("cam_sensor", sum(a[1] for a in srl.avio() if a[0] in ("camera", "sensor")),
                 geo.x_parts["cam_sensor"], 0.0),
        MassItem("fc_esc", sum(a[1] for a in srl.avio() if a[0] in ("fc", "rx_vtx")),
                 geo.x_parts["fc_esc"], 0.0),
    ]
    m_tot = sum(i.m for i in bd)
    x_cg = sum(i.m * i.x for i in bd) / m_tot

    # §5 3축 관성 (x=롤, y=피치, z=요 · 축대칭 → J_yy=J_zz)
    J_xx = sum(i.m * i.r ** 2 for i in bd)
    J_shell_len = st.breakdown_str[0].m * geo.l_body ** 2 / 12.0   # 동체 쉘만 길이항
    J_yy = sum(i.m * ((i.x - x_cg) ** 2 + 0.5 * i.r ** 2) for i in bd) + J_shell_len
    J_zz = J_yy

    return WghtOut(MTOW=m_tot, x_cg=x_cg, J_xx=J_xx, J_yy=J_yy, J_zz=J_zz,
                   breakdown=bd, converged=converged, n_iter=it, strc=st)
