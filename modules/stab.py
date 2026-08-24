"""STAB — 정적안정 · 천이 조종성.  ICD0-008 §5.1

⚠ 이 구조의 주 병목 — 요구 성능 조건들이 사이징으로 흡수되면서 g8·g9 가 주 탈락
   사유가 될 전망이다. S_fin·x_fin 범위 손질이 스크리닝의 우선순위다.

⚠ M_dist 를 상수로 두지 않는다 (§4.4). 물리적으로 풍하중이므로 동압과 형상의 함수다:
       M_dist = q · S_ref · CN_α · Δα · |x_cp − x_cg|
   상수 하나에 g8·g9 가 달려 있으면 DOE 결과 전체가 그 상수에 좌우된다.

[스텁] P6 에서 구현한다.
[결정 필요] Δα(기준 돌풍 받음각) — 운용 환경 가정에서 도출 (ICD §8 A-4)
"""
import constants as k
from interfaces import HullOut, AeroOut, LayoutOut, MassProps, StabOut


def run(hl: HullOut, aer: AeroOut, mp: MassProps, lay: LayoutOut,
        T_hover: float) -> StabOut:
    """② 정적안정 · 천이 조종성.

    [스텁] 구현 예정 (P6):
        SM = (x_cp − x_cg) / d_body           → g8 : SM ≥ SM_min
        M_dist = q·S_ref·CN_α·Δα·|x_cp − x_cg|   (상수가 아니라 계산값)
        M_ctrl = 차동 추력 × arm_rotor
        alpha_max = M_ctrl / J_yy             ← EC C5
                                              → g9 : M_ctrl / M_dist ≥ k_ctrl
    """
    return StabOut(
        SM=0.0,             # [스텁]
        alpha_max=0.0,      # [스텁] ← EC C5
        g8=0.0,             # [스텁] 실제 안정 판정 아님
        g9=0.0,             # [스텁] 실제 조종 판정 아님
        M_dist=0.0,         # [스텁]
        M_ctrl=0.0,         # [스텁]
    )
