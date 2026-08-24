"""COST — 취득비.  ICD0-008 §5.1

정밀도: C7 가중치가 4.2% 라 대표값 수준이면 충분하다.
여기에 정확도 예산을 쓸 자리가 아니다.

[스텁] P6 에서 구현한다 — 단가 계수가 전부 TBD(§8 B-1)라 값이 없다.
"""
import constants as k
from interfaces import CostOut


def run(m_mot: float, P_cont: float, E_batt: float, d_prop: float,
        I_max: float, m_print: float) -> CostOut:
    """② 단가 계수 × 각 항 합산.

    [스텁] 구현 예정 (P6):
        모터        : c_mot_krw  × 연속출력
        배터리      : c_batt_krw × E_batt
        프롭        : c_prop_krw × 기수(N_rot)
        ESC         : c_esc_krw  × 요구 정격(I_max × k_esc_margin)
        프린트 재료 : m_print × c_filament_krw
        항전        : AVIO_LIST 단가 합
    """
    bd = {
        "motor": 0.0,     # [스텁]
        "batt": 0.0,      # [스텁]
        "prop": 0.0,      # [스텁]
        "esc": 0.0,       # [스텁]
        "print": 0.0,     # [스텁]
        "avio": 0.0,      # [스텁]
    }
    return CostOut(Cost_acq=sum(bd.values()), breakdown=bd)   # [스텁] ← EC C7
