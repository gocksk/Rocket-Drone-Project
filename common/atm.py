"""ATM — 표준대기 공용 함수. [실제 구현]
가이드라인: 「ATM 가이드라인 — 표준대기」
모든 모듈은 공기 물성을 여기서만 받는다. 하드코딩 금지 (ICD §7).
ΔT 인수는 앵커 측정 조건 재현용 — 설계점 평가는 전부 ΔT=0.
"""
from dataclasses import dataclass
import math

T0, P0, L, R, GAMMA, G0 = 288.15, 101325.0, 0.0065, 287.05, 1.4, 9.80665


@dataclass
class Air:
    rho: float   # 밀도 [kg/m^3]
    T: float     # 온도 [K]
    p: float     # 압력 [Pa]
    a_snd: float # 음속 [m/s]
    mu: float    # 점성 [Pa·s]


def atm(h: float = 0.0, dT: float = 0.0) -> Air:
    T_std = T0 - L * h
    p = P0 * (1.0 - L * h / T0) ** (G0 / (L * R))
    T = T_std + dT
    rho = p / (R * T)
    a_snd = math.sqrt(GAMMA * R * T)
    mu = 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)
    return Air(rho, T, p, a_snd, mu)


if __name__ == "__main__":  # 검산 (가이드라인 §1·§2)
    # `python common/atm.py`는 sys.path[0]이 common/이라 저장소 루트를 먼저 얹는다.
    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
    from common.out import stdout_utf8
    stdout_utf8()

    a = atm(0.0, 0.0)
    assert abs(a.rho - 1.225) < 1e-3, a.rho
    assert abs(a.a_snd - 340.3) < 0.1, a.a_snd
    assert abs(a.mu - 1.789e-5) < 2e-8, a.mu
    print("ATM 검산 통과:", a)
