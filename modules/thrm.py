"""THRM — 모터 열.  ICD0-008 §5.1

호버 가능 여부의 실제 한계는 '순간 출력'이 아니라 모터가 타지 않는가다.
PROP.size_motor 가 이분법 안쪽에서 직접 부른다 (§5 유일한 예외).

[구조 완료 — 계수 미확정]
덩어리 열용량 1차 ODE 와 표면적 환산은 구현했다. 대류 상관식 파라미터
(Nu_C_*, Nu_m_*), c_mot, ar_mot, T_limit 이 전부 TBD 라 **절대 온도값은
보증되지 않는다.** 반응 방향(동손↑ → 온도↑, 모터 크기↑ → 온도↓)만 믿을 수 있다.

⚠ 설계상 주의 세 가지 — 구현이 셋 다 반영했다
  1. ICD 는 최악 조건을 **착륙 호버**로 못박았다. 그런데 **이 기체에서는 순항이
     더 뜨겁게 나온다** — 300 km/h 순항이 1.4 kW 를 먹고 호버는 그 1/7 이라
     동손 비(5.8)가 대류 비(5.2)를 넘는다. 그래서 어느 쪽이 이기는지 판정하지 않고
     **둘 다 계산해 최대값**을 쓴다 (T_hot·hot_at). [§11-17]
     참고: ICD 의 "저전압 → 고전류" 논리는 **팩 전류**에 대한 것이다. 모터 동손은
     I=τ/K_t+I0 라 토크가 정하고 버스 전압과 무관하다 — 확인 필요
  2. **초기 온도는 순항 정상상태 온도**다. 외기로 두면 과소평가다 —
     착륙 시점 모터는 이미 순항으로 데워져 있다
  3. **대류계수를 구간별로 따로** 잡는다. 순항 300 km/h 는 강제대류(Re 기반 Nu),
     호버는 프롭 후류. 호출부가 각 구간의 유속을 넘긴다

[결정 필요] c_mot · Nu_corr · ar_mot · T_limit(권선 절연 등급) — ICD §8 A-3
"""
import math

import constants as k
from interfaces import AtmOut, ThrmOut


def motor_geometry(m_mot: float) -> tuple:
    """모터 질량 → (외경 D [m], 길이 L [m], 표면적 A [m²]).

    부피 = m_mot / rho_mot, 형상비 ar_mot = D/L 을 원통에 걸어 역산한다:
        V = π·D²/4 · L = π·D³/(4·ar_mot)   →   D = (4·ar_mot·V/π)^(1/3)
    표면적은 측면 + 양 끝면. 방열핀·벨 형상은 무시했다 (보수적).
    """
    vol = m_mot / k.rho_mot
    D = (4.0 * k.ar_mot * vol / math.pi) ** (1.0 / 3.0)
    L = D / k.ar_mot
    A = math.pi * D * L + 0.5 * math.pi * D * D
    return D, L, A


def _hA(V_air: float, D: float, A: float, air: AtmOut, C: float, m: float) -> float:
    """**권선 → 외기** 실효 열컨덕턴스 [W/K].

        1/G = 1/(h·A) + R_int_mot        (표면 대류와 내부 전도를 직렬로)
        h·A : 원통 횡류 Hilpert 형 Nu = C·Re^m,  Pr^(1/3) 은 C 에 흡수

    내부 항이 없으면 모터 전체가 등온이라는 뜻이 되어 권선 온도를 계통적으로
    과소평가하고, 그 결과 사이징이 모터를 실제보다 작게 뽑는다. 열질량은 여전히
    하나이므로 §5.1 의 '덩어리 열용량 1차 ODE' 구조는 그대로다.
    """
    Re = max(V_air, 1e-3) * D / air.nu
    Nu = C * Re ** m
    hA = Nu * k.k_air / D * A
    return 1.0 / (1.0 / max(hA, 1e-12) + k.R_int_mot)


def motor_rise(P_cu_cruise: float, P_cu_hover: float, m_mot: float,
               V_cruise: float, V_wake: float, t_hover: float,
               air: AtmOut) -> ThrmOut:
    """동손 → 권선 온도 상승. 덩어리(lumped) 열용량 1차 미분방정식.

        m_mot · c_mot · dT/dt = P_cu − h·A·(T − T_amb)

    P_cu_cruise / P_cu_hover : **모터 1기당** 동손 [W]
    V_cruise                 : 순항 비행속도 [m/s] — 강제대류
    V_wake                   : 호버 프롭 후류 속도 [m/s] — 호출부가 유도속도로 낸다
    t_hover                  : 호버 지속시간 [s] — 착륙 세그먼트 기준

    순항은 정상상태 해, 호버는 그 온도를 초기값으로 한 지수 상승이다.
    """
    D, _, A = motor_geometry(m_mot)

    # 순항 — 정상상태 (임무 대부분이 순항이라 과도항이 다 죽는다)
    hA_cr = _hA(V_cruise, D, A, air, k.Nu_C_cruise, k.Nu_m_cruise)
    T_cruise_ss = k.T_amb + P_cu_cruise / max(hA_cr, 1e-12)

    # 호버 — 지수 상승. 초기값은 순항 정상상태 (주의 2)
    hA_hv = _hA(V_wake, D, A, air, k.Nu_C_hover, k.Nu_m_hover)
    T_inf = k.T_amb + P_cu_hover / max(hA_hv, 1e-12)     # 호버를 무한히 하면 가는 온도
    tau = m_mot * k.c_mot / max(hA_hv, 1e-12)            # 시정수 [s]
    T_peak = T_inf + (T_cruise_ss - T_inf) * math.exp(-t_hover / max(tau, 1e-12))

    # 더 뜨거운 쪽이 한계를 정한다.  [로컬 개정 §11-17]
    # ICD §5.1 은 "최악 조건은 착륙 호버" 를 전제로 열 여유를 T_peak 로만 적었다.
    # 그건 호버가 지속 고출력인 통상 멀티로터의 이야기다. 이 기체는 300 km/h 순항이
    # 1.4 kW 를 먹고 호버는 그 1/7 이라 **동손 비가 대류 비를 넘어** 순항 정상상태가
    # 더 뜨거울 수 있다. 그러면 호버는 오히려 모터를 식힌다.
    # 어느 쪽이 이기는지는 설계점마다 다르므로 최대값으로 판정한다.
    if T_cruise_ss >= T_peak:
        T_hot, hot_at = T_cruise_ss, "cruise"
    else:
        T_hot, hot_at = T_peak, "hover"

    return ThrmOut(T_cruise_ss=T_cruise_ss, T_peak=T_peak,
                   T_hot=T_hot, hot_at=hot_at, margin_T=k.T_limit - T_hot)


if __name__ == "__main__":   # 검산 — 극한 거동이 물리적으로 맞는가
    from common.out import stdout_utf8
    from modules import atm
    stdout_utf8()
    air = atm.run(0.0)

    m = 0.034
    D, L, A = motor_geometry(m)
    print(f"모터 34 g → D={D*1e3:.1f} mm  L={L*1e3:.1f} mm  A={A*1e4:.2f} cm²")
    print(f"  (실제 2207 급이 D≈28 mm — 형상비 ar_mot={k.ar_mot} 가정의 결과)")

    hA_cr = _hA(k.V_cr, D, A, air, k.Nu_C_cruise, k.Nu_m_cruise)
    hA_hv = _hA(8.7, D, A, air, k.Nu_C_hover, k.Nu_m_hover)
    print(f"  h·A 순항(83.3 m/s) = {hA_cr:.3f} W/K   호버(후류 8.7 m/s) = {hA_hv:.3f} W/K"
          f"   비 {hA_cr/hA_hv:.1f}배")
    print(f"  호버 시정수 τ = {m*k.c_mot/hA_hv:.0f} s")

    r = motor_rise(40.0, 12.0, m, k.V_cr, 8.7, 20.0, air)
    print(f"\n동손 순항 40 W · 호버 12 W · 착륙 호버 20 s:")
    print(f"  T_cruise_ss={r.T_cruise_ss:.1f} °C  T_peak={r.T_peak:.1f} °C"
          f"  열 여유={r.margin_T:.1f} K")

    # (1) 초기 온도를 외기로 두면 과소평가인가 — ICD 주의 2 의 크기
    hA = hA_hv
    T_inf = k.T_amb + 12.0 / hA
    tau = m * k.c_mot / hA
    T_naive = T_inf + (k.T_amb - T_inf) * math.exp(-20.0 / tau)
    print(f"  초기값을 외기로 두면 {T_naive:.1f} °C → **{r.T_peak - T_naive:.1f} K 과소평가**")

    # (2) 단조성
    a = motor_rise(40.0, 12.0, 0.020, k.V_cr, 8.7, 20.0, air).T_peak
    b = motor_rise(40.0, 12.0, 0.060, k.V_cr, 8.7, 20.0, air).T_peak
    print(f"  모터 20 g → {a:.1f} °C,  60 g → {b:.1f} °C   (커질수록 시원해야 한다)")
    assert b < a, "모터를 키웠는데 더 뜨겁다"
    c = motor_rise(40.0, 24.0, m, k.V_cr, 8.7, 20.0, air).T_peak
    assert c > r.T_peak, "동손을 늘렸는데 더 시원하다"
    assert r.T_peak > k.T_amb, "발열이 있는데 외기보다 차갑다"
    # (3) t_hover → 0 이면 순항 정상상태 그대로
    z = motor_rise(40.0, 12.0, m, k.V_cr, 8.7, 0.0, air)
    assert abs(z.T_peak - z.T_cruise_ss) < 1e-12, "t=0 에서 초기값과 달라진다"

    # (4) 어느 구간이 뜨거운지 판정이 뒤집히는가 — 호버가 모터를 식히는 경우
    hot_hv = motor_rise(20.0, 30.0, m, k.V_cr, 8.7, 20.0, air)   # 호버 동손이 크면
    hot_cr = motor_rise(60.0, 3.0, m, k.V_cr, 8.7, 20.0, air)    # 순항 동손이 크면
    print(f"  동손 순항20/호버30 → 뜨거운 쪽 '{hot_hv.hot_at}' (T={hot_hv.T_hot:.1f}°C)")
    print(f"  동손 순항60/호버 3 → 뜨거운 쪽 '{hot_cr.hot_at}' (T={hot_cr.T_hot:.1f}°C, "
          f"호버가 오히려 {hot_cr.T_cruise_ss - hot_cr.T_peak:.1f} K 식힌다)")
    assert hot_hv.hot_at == "hover" and hot_cr.hot_at == "cruise", "구간 판정이 안 뒤집힌다"
    print("THRM 검산 통과")
