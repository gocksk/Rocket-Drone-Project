"""STAB — 정적안정 · 천이 조종성.  ICD0-008 §5.1

⚠ 이 구조의 주 병목 — 요구 성능 조건들이 사이징으로 흡수되면서 g8·g9 가 주 탈락
   사유가 될 전망이다. S_fin·x_fin 범위 손질이 스크리닝의 우선순위다.

⚠ M_dist 를 상수로 두지 않는다 (§4.4). 물리적으로 풍하중이므로 동압과 형상의 함수다:
       M_dist = q · S_ref · CN_α · Δα · |x_cp − x_cg|
   상수 하나에 g8·g9 가 달려 있으면 DOE 결과 전체가 그 상수에 좌우된다.

[구조 완료 — 계수 미확정]
`Δα`(기준 돌풍 받음각) · `k_ctrl` · `k_az`(방위각 규약) 가 TBD 다.
반응 방향만 믿을 수 있다.
"""
import constants as k
from interfaces import HullOut, AeroOut, LayoutOut, MassProps, StabOut


def run(dv, hl: HullOut, aer: AeroOut, mp: MassProps, lay: LayoutOut,
        dT_rotor: float) -> StabOut:
    """② 정적안정 · 천이 조종성.

    dT_rotor : 로터 1기가 **더** 낼 수 있는 추력 [N]. 지금 쓰고 있는 작동점 위의
        여유이며 런처가 PROP 으로 계산해 넘긴다 (STAB→PROP 은 새 모듈 간 호출이라
        금지, §5). 이 값이 조종 권한의 원천이다.

    계산 (§5.1)
        SM = (x_cp − x_cg) / d_body                → g8 : SM ≥ SM_min
        M_dist = q·S_ref·CN_α·Δα·|x_cp − x_cg|      — 상수가 아니라 계산값 (§4.4)
        M_ctrl = k_az · ΔT · arm_rotor              — 차동 추력 × 모멘트암
        alpha_max = M_ctrl / J_yy                   ← EC C5
                                                    → g9 : M_ctrl/M_dist ≥ k_ctrl
    """
    # ── 정적 안정 ──
    SM = (aer.x_cp - mp.x_cg) / dv.d_body            # [칼리버]
    g8 = SM - k.SM_min

    # ── 외란 모멘트 — 풍하중 (§4.4) ──
    M_dist = (aer.q_cr * aer.S_ref * aer.CN_alpha * k.d_alpha
              * abs(aer.x_cp - mp.x_cg))

    # ── 조종 모멘트 — 차동 추력 ──
    M_ctrl = k.k_az * max(dT_rotor, 0.0) * lay.arm_rotor
    alpha_max = M_ctrl / max(mp.J_yy, 1e-12)         # [rad/s²]  ← C5
    g9 = M_ctrl / max(M_dist, 1e-12) - k.k_ctrl

    return StabOut(SM=SM, alpha_max=alpha_max, g8=g8, g9=g9,
                   M_dist=M_dist, M_ctrl=M_ctrl)
