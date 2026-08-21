"""SRL — 부품 스펙 회귀 라이브러리. [스텁]
가이드라인: 「SRL 가이드라인 — 스펙 회귀 라이브러리」

[스텁] 표시된 수식은 전부 임의의 잠정 회귀다. 조사 담당이 스펙표
10~15종을 모아 진짜 회귀 계수로 교체한다 (계수는 데이터 파일로 분리 권장).
반환값은 전부 SI (m, kg, Ω, A, V). 외삽 금지 규약은 교체 시 구현할 것.
"""
from dataclasses import dataclass
import constants as k


@dataclass
class MotorSpec:
    m: float; R_mot: float; I_0: float; I_limit: float
    L: float; D: float; price: float


@dataclass
class PropSpec:
    m: float; price: float


@dataclass
class EscSpec:
    m: float; I_rated: float; L: float; W: float; H: float
    l_esc: float; price: float


@dataclass
class BattSpec:
    m: float; R_pack: float; c_rate: float
    L: float; W: float; H: float; price: float


def motor(kv_mot, d_stat, h_stat) -> MotorSpec:
    vol = d_stat ** 2 * h_stat                      # 스테이터 체적 지표
    m = 5300.0 * vol                                # [스텁]
    R_mot = 100.0 / kv_mot                          # [스텁] R ∝ 1/kv 근사
    I_0 = 0.0010 * kv_mot                           # [스텁]
    I_limit = 6.0e6 * vol                           # [스텁]
    return MotorSpec(m, R_mot, I_0, I_limit,
                     L=h_stat + 0.012, D=d_stat + 0.004,
                     price=9.0e8 * vol)             # [스텁]


def prop(d_prop, pd_prop) -> PropSpec:
    m = 1.5 * d_prop ** 2.7                         # [스텁]
    return PropSpec(m, price=8000.0)                # [스텁]


def esc(I_req) -> EscSpec:
    I_rated = I_req                                  # 정격은 호출부에서 마진 적용
    m = 0.0003 * I_rated + 0.006                    # [스텁]
    return EscSpec(m, I_rated, L=0.030, W=0.022, H=0.006,
                   l_esc=0.030, price=400.0 * I_rated)   # [스텁]


def batt(E_batt, n_ser) -> BattSpec:
    m = E_batt / k.e_spec                           # 정의상 이 식 (WGHT와 동일 출처)
    R_pack = 0.010 * n_ser * (50.0 / max(E_batt, 1.0))   # [스텁]
    vol_L = E_batt / 500.0                          # [스텁] 500 Wh/L
    W, H = 0.036, 0.038
    L = vol_L * 1e-3 / (W * H)
    return BattSpec(m, R_pack, c_rate=k.c_rate_max, L=L, W=W, H=H,
                    price=900.0 * E_batt)           # [스텁]


def U_ocv(SOC) -> float:
    """셀당 개로전압 — 3점 선형 (SRL §4)."""
    if SOC >= 0.5:
        return 3.7 + (4.2 - 3.7) * (SOC - 0.5) / 0.5
    return 3.4 + (3.7 - 3.4) * SOC / 0.5


def avio() -> list:
    """항전 고정 목록 — [스텁] 품목·값은 항전 담당이 확정.
    (name, m[kg], L, W, H, price[KRW])"""
    return [
        ("camera", 0.012, 0.020, 0.019, 0.019, 25000.0),
        ("fc",     0.010, 0.030, 0.030, 0.008, 60000.0),
        ("rx_vtx", 0.008, 0.025, 0.015, 0.006, 40000.0),
        ("sensor", 0.015, 0.030, 0.020, 0.012, 50000.0),
    ]
