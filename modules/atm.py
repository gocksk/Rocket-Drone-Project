"""ATM — 표준대기.  ICD0-008 §5.1

[구조 완료 · 검증 완료] 스텁이 아니다.
기존 common/atm.py 의 검증된 구현을 ICD0-008 시그니처 run(h) 로 옮긴 것이다.
검산은 아래 __main__ 블록 — h=0 에서 ρ=1.225, a=340.3, μ=1.789e-5.

종속성: 무게·설계변수 모두와 무관 → ⓪에서 1회만 실행한다.
"""
import math

from interfaces import AtmOut

# ISA 대류권 기준값 — 물리 정의값이라 constants.py 가 아니라 여기 둔다.
T0 = 288.15        # 해면 온도 [K]
P0 = 101325.0      # 해면 압력 [Pa]
L = 0.0065         # 기온감률 [K/m]
R = 287.05         # 건공기 기체상수 [J/kg·K]
GAMMA = 1.4        # 비열비 [-]
G0 = 9.80665       # 표준중력 [m/s²]


def run(h: float = 0.0) -> AtmOut:
    """고도 → 공기 물성. ISA 대류권 (h ≤ 11 km)."""
    T = T0 - L * h
    # 정수압 평형 dp/dh = -ρg 를 선형 온도분포에서 적분한 해
    p = P0 * (1.0 - L * h / T0) ** (G0 / (L * R))
    rho = p / (R * T)
    a_snd = math.sqrt(GAMMA * R * T)
    # Sutherland 식 — 온도만의 함수
    mu = 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)
    return AtmOut(rho=rho, T=T, a_snd=a_snd, nu=mu / rho, p=p, mu=mu)


if __name__ == "__main__":   # 검산 — ICD §5.1 ATM "h=0 에서 ρ = 1.225"
    from common.out import stdout_utf8
    stdout_utf8()
    a = run(0.0)
    assert abs(a.rho - 1.225) < 1e-3, a.rho
    assert abs(a.a_snd - 340.3) < 0.1, a.a_snd
    assert abs(a.mu - 1.789e-5) < 2e-8, a.mu
    print("ATM 검산 통과:", a)
