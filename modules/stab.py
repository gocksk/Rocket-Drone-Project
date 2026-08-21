"""STAB — 안정성 검사. [구조 실제 구현]
가이드라인: 「STAB 계산 가이드라인 — 안정성 검사」
M_avail은 여기서 계산한다 (다른 모듈은 재료만 제공 — ICD §5).
x_cp(α) 가중평균은 AERO의 것을 호출한다 — 다시 정의하지 말 것.
"""
import math
import constants as k
from interfaces import DesignVars, StabOut


def run(dv: DesignVars, geo, aero, wg, pb) -> StabOut:
    # §2 정적 안정
    SM = (aero.x_cp - wg.x_cg) / dv.d_body
    g6 = SM / k.SM_min - 1.0

    # §3 조종 모멘트 — 피치·요: 차동추력, 롤: 반동토크
    M_pitch = k.k_az * pb.dT_max * geo.arm_rotor      # k_az: X배치 2√2 (승인 대기)
    M_roll = 2.0 * pb.dQ_max

    # §4 조종 민첩성 (EC C5)
    alpha_max = M_pitch / max(wg.J_yy, 1e-9)

    # §5 천이 복원 모멘트 — 받음각 격자 스캔 (최악점은 중간 어딘가)
    q_trans = k.k_q * aero.q_cr
    M_aero_max, a_worst = 0.0, 0.0
    a = 0.0
    while a <= math.pi / 2 + 1e-9:
        M = q_trans * geo.S_ref * aero.C_N(a) * (aero.x_cp_alpha(a) - wg.x_cg)
        if abs(M) > M_aero_max:
            M_aero_max, a_worst = abs(M), a
        a += k.d_alpha
    r_ctrl = M_pitch / max(M_aero_max, 1e-9)
    g7 = r_ctrl / k.k_ctrl - 1.0

    return StabOut(SM=SM, r_ctrl=r_ctrl, M_avail_pitch=M_pitch,
                   M_avail_roll=M_roll, alpha_max=alpha_max, g6=g6, g7=g7)
