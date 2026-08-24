"""STRC — 프린트 구조 무게.  ICD0-008 §5.1

MTOW 에 반응하는 순수 구조 항이다. 이전 구조에서는 수렴 루프의 유일한 응답이었고,
지금은 셋(구조·모터·배터리) 중 하나다.

⚠ 슬라이서 출력은 레이어 수·둘레 수가 정수라 계단형일 수 있다. 그러면 MTOW 가 두 값
   사이를 진동하는데, 이건 발산이 아니라 limit_cycle 로 분류된다 (WGHT).

[스텁] P6 에서 구현한다.
"""
import constants as k
from interfaces import DesignVars, HullOut, AeroOut, StrcOut, MassItem

# [스텁] 가짜 기울기 — 슬라이서 회귀(k_sl_*)가 실측 대기라 '정답'을 모른다.
# 0 을 반환하면 응답 질량이 MTOW 에 반응하지 않아 ① 수렴 루프가 무의미해지므로
# 배관 확인용으로 선형 기울기 하나만 넣어 둔다. **물리적 근거가 없다.**
_STUB_SLOPE = 0.30      # [스텁] W_str = _STUB_SLOPE × MTOW


def run(dv: DesignVars, hl: HullOut, aer: AeroOut, MTOW: float) -> StrcOut:
    """① 프린트 구조 무게 — 설계 하중에서 구조를 낸다.

    [스텁] 구현 예정 (P6):
        설계 하중 = n_design × MTOW + 공력 하중(핀·동체)
        슬라이서 회귀 : 인필율 · 벽두께 · 둘레 수 → 무게
        응력 검산으로 g5 여유 산출
    """
    W_str = _STUB_SLOPE * MTOW      # [스텁] 실제 구조 무게 아님

    # breakdown_str[0] 은 WGHT 가 동체 쉘로 쓴다(길이항 J) — 최소 한 항목은 있어야 한다.
    bd = [MassItem("shell", W_str, 0.5 * hl.l_body, 0.0)]   # [스텁] 위치도 자리표시

    return StrcOut(
        W_str=W_str,
        m_print=W_str,      # [스텁] 프린트 재료량 = 구조 무게로 둔 자리표시
        g5=0.0,             # [스텁] 실제 응력 판정 아님
        breakdown_str=bd,
    )
