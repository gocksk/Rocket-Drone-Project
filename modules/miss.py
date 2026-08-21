"""MISS — 임무 시뮬레이션. [구조 실제 구현]
가이드라인: 「MISS 계산 가이드라인 — 임무 시뮬레이션」
단위 규약: 입력 E_batt만 Wh, 내부는 전부 J (1 Wh = 3600 J).
착륙분(SOC_land)을 남기고 멈춘다 — 안 그러면 도착 즉시 추락 설계가 최적해로 나온다.
"""
import math
import constants as k
from interfaces import DesignVars, MissOut


def run(dv: DesignVars, geo, aero, pa, pb, MTOW) -> MissOut:
    W = MTOW * k.g
    E_J = dv.E_batt * 3600.0
    E_usable = E_J * k.DoD
    U = pa.U_eval

    # 구간 1 — 수직이륙·상승
    t_1 = k.h_to / k.V_climb
    T_11 = W / (k.N_rot * k.k_block)
    P1, _, _ = pa.query(k.V_climb, T_11, U)
    E_1 = k.N_rot * P1 * t_1

    # 구간 2 — 천이·가속 (가속 운동에너지 명시 — 승인 대기 항)
    E_2 = pb.P_hover * k.k_trans * k.t_trans \
        + 0.5 * MTOW * k.V_cr ** 2 / k.eta_acc

    # 구간 4 — 재천이·감속 (운동에너지 항 없음 — 비대칭)
    E_4 = pb.P_hover * k.k_trans * k.t_trans

    # 구간 5 — 수직착륙
    t_5 = k.h_to / k.V_desc
    P5, _, _ = pa.query(k.V_desc, T_11, U)
    E_5 = k.N_rot * P5 * t_5

    E_fixed = E_1 + E_2 + E_4 + E_5
    seg = {"1_climb": E_1, "2_trans": E_2, "4_retrans": E_4, "5_land": E_5}

    if E_usable <= E_fixed:
        return MissOut(R_dash=0.0, E_req=E_fixed, t_mission=t_1 + 2 * k.t_trans + t_5,
                       seg_energy=seg, g8=-1.0)

    # 구간 3 — 고속비행 시간전진
    T_req1 = math.sqrt(aero.F_drag(k.V_cr) ** 2 + W ** 2) / k.N_rot
    SOC = 1.0 - (E_1 + E_2) / E_J
    SOC_land = 1.0 - k.DoD + (E_4 + E_5) / E_J
    R, t, I_prev, E_3 = 0.0, 0.0, 20.0, 0.0
    while SOC > SOC_land:
        U_now = pa.U_pack(SOC, I_prev)
        P, I, _ = pa.query(k.V_cr, T_req1, U_now)
        E_step = k.N_rot * P * k.dt_march
        SOC -= E_step / E_J
        R += k.V_cr * k.dt_march
        t += k.dt_march
        E_3 += E_step
        I_prev = k.N_rot * I / k.eta_esc
        if t > 3600.0:                      # 안전장치
            break

    seg["3_dash"] = E_3
    g8 = R / k.R_dash_min - 1.0
    return MissOut(R_dash=R, E_req=E_fixed + E_3,
                   t_mission=t_1 + 2 * k.t_trans + t_5 + t,
                   seg_energy=seg, g8=g8)
