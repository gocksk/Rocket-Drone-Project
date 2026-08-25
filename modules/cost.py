"""COST — 취득비.  ICD0-008 §5.1

정밀도: C7 가중치가 4.2% 라 대표값 수준이면 충분하다.
여기에 정확도 예산을 쓸 자리가 아니다.

[구조 완료 — 단가 전부 TBD]
합산 구조는 §5.1 그대로다. 단가 계수 5종이 전부 미확정(§8 B-1)이라 **금액의 절대값은
의미가 없다.** 설계점 사이의 상대 비교만 방향이 맞다.
"""
import constants as k
from interfaces import CostOut


def run(m_mot: float, P_cont: float, E_batt: float, d_prop: float,
        I_max: float, m_print: float) -> CostOut:
    """② 단가 계수 × 각 항 합산.

    P_cont : 연속 출력 [W] — 로터 기수 합. 모터 단가의 기준
    I_max  : 임무 중 최대 팩 전류 [A] — ESC 정격의 기준
    """
    bd = {
        "motor": k.c_mot_krw * (P_cont / 1000.0),           # KRW/kW × kW
        "batt": k.c_batt_krw * E_batt,                      # KRW/Wh × Wh
        "prop": k.c_prop_krw * k.N_rot,                     # KRW/기 × 기수
        "esc": k.c_esc_krw * I_max * k.k_esc_margin,        # KRW/A × 요구 정격
        "print": k.c_filament_krw * m_print,                # KRW/kg × kg
        "avio": sum(a[5] for a in k.AVIO_LIST),             # 목록 합
    }
    return CostOut(Cost_acq=sum(bd.values()), breakdown=bd)
