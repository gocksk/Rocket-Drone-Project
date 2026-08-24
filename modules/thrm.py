"""THRM — 모터 열.  ICD0-008 §5.1

호버 가능 여부의 실제 한계는 '순간 출력'이 아니라 모터가 타지 않는가다.
PROP.size_motor 가 이분법 안쪽에서 직접 부른다 (§5 유일한 예외).

[스텁] P4 에서 구현한다.

⚠ 설계상 주의 세 가지 (구현 시 반드시 지킬 것)
  1. 최악 조건은 **착륙 호버**다 — 방전 말기라 전압이 낮고, 같은 추력에 전류가
     커지며, 동손은 전류의 제곱이다. U_eval 규약과 같은 논리.
  2. **초기 온도를 외기로 두면 과소평가**다 — 착륙 시점 모터는 이미 순항으로
     데워져 있다. 순항 정상상태 온도를 초기값으로 받는다.
  3. **대류계수가 구간별로 완전히 다르다** — 순항 300 km/h 는 강제대류(Re 기반 Nu),
     호버는 프롭 후류. 상관식을 따로 잡는다.

[결정 필요] 호버 지속시간 정의(천이 t_trans 기준인가, 이륙·착륙 각각인가) ·
            T_limit(권선 절연 등급) · c_mot · Nu_corr · ar_mot (ICD §8 A-3)
"""
import constants as k
from interfaces import AtmOut, ThrmOut


def motor_rise(P_cu_cruise: float, P_cu_hover: float, m_mot: float,
               V_cruise: float, V_wake: float, t_hover: float,
               air: AtmOut) -> ThrmOut:
    """동손 → 권선 온도 상승. 덩어리(lumped) 열용량 1차 미분방정식.

        m_mot · c_mot · dT/dt = P_cu − h·A·(T − T_amb)

      · 표면적 A 를 m_mot 과 ar_mot 에서 환산
      · 순항 : 정상상태 해  T_ss = T_amb + P_cu/(h·A)
      · 호버 : T_cruise_ss 를 초기값으로 지수 상승, 호버 시간 후가 T_peak

    [스텁] 위 식과 대류계수 상관식이 아직 없다.
    """
    return ThrmOut(
        T_cruise_ss=k.T_amb,        # [스텁] 실제 정상상태 온도 아님
        T_peak=k.T_amb,             # [스텁] 실제 피크 온도 아님
        margin_T=k.T_limit - k.T_amb,   # [스텁] 위가 가짜라 이것도 가짜다
    )
