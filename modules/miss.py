"""MISS — 임무 적분 · 배터리 사이징 · 달성 거리.  ICD0-008 §5.1

적분기는 **하나**다. required_energy(사이징용)와 achieved_range(평가용)는
같은 커널 integrate 의 다른 진입점일 뿐이다 (§4.6 이중 구현 금지).

⚠ 고정 스텝 필수 — 적응 스텝은 잔차에 인공 노이즈를 만들어 WGHT 의
   resid_floor·양자화 판정 논리를 흐린다.

[스텁] P5 에서 구현한다.
"""
import constants as k
from interfaces import (DesignVars, AtmOut, AeroOut, PropMapOut,
                        MissHistory, RequiredEnergyOut, AchievedRangeOut)


def integrate(profile: list, MTOW: float, m_mot: float, E_batt: float,
              n_ser: int, pmap: PropMapOut, aer: AeroOut, air: AtmOut,
              SOC_0: float = 1.0, until_depleted: bool = False) -> MissHistory:
    """공용 커널 — 고정 스텝 dt_miss 전진. 매 스텝마다:

        prop.solve_point 호출 → P → I → SOC 감소 → U_ocv(SOC)로 버스 전압 갱신 → 거리 누적

    전압이 떨어지면 같은 추력에 전류가 커지는 말기 악화가 자연히 반영된다.

    until_depleted=True 면 프로파일 대신 SOC 소진까지 dash 를 계속한다
    (achieved_range 진입점).

    [스텁] P5 에서 구현한다.
    """
    return MissHistory(
        t=[0.0], x=[0.0], E=[0.0], SOC=[SOC_0],     # [스텁] 실제 이력 아님
        U_bus=[0.0],                                # [스텁]
        I_max=0.0,                                  # [스텁]
        depleted=False,                             # [스텁]
    )


def required_energy(MTOW: float, m_mot: float, dv: DesignVars,
                    pmap: PropMapOut, aer: AeroOut, air: AtmOut) -> RequiredEnergyOut:
    """① 요구 임무 실적분 → 배터리 사이징.  E_batt 에 대한 **결정론적 이분법**.

    후보 용량마다:
      · m_batt = E_batt / e_spec,  R_pack = k_Rpack · n_ser / cap
      · integrate 실행 → 요구 거리 도달 시점의 SOC 여유
      · SOC 여유가 0(DoD 한계)이 되는 최소 용량 → E_energy
      · 전류 요구에서 E_power = I_max / c_rate_max
      · E_batt = k_E × max(E_energy, E_power)

    ⚠ 내부 순환(E_batt → R_pack → 전압 → 전류 → 소비 에너지 → E_batt)을
       직전 반복값으로 닫지 않는다. 이분법만 순수성을 지킨다.

    [스텁] P5 에서 구현한다.
    """
    E_batt = 0.0                                # [스텁] 실제 요구 용량 아님
    m_batt = E_batt / k.e_spec                  # 정의식 (질량 환산)
    m_pack = k.k_pack * m_batt                  # 정의식 (팩 부자재)
    return RequiredEnergyOut(
        E_batt=E_batt, m_batt=m_batt, m_pack=m_pack,
        active="stub",      # [스텁] energy/power 판별 미구현
        n_bisect=0,
    )


def achieved_range(MTOW: float, m_mot: float, E_batt: float, dv: DesignVars,
                   pmap: PropMapOut, aer: AeroOut, air: AtmOut) -> AchievedRangeOut:
    """② 확정 스펙으로 달성 거리 — 같은 커널을 SOC 소진까지 돌려 누적 거리를 읽는다.

    k_E > 1 이면 R_dash > R_dash_min 이 된다.

    [스텁] P5 에서 구현한다. 커널 공유가 완료 판정이다 — 여기서 적분을 다시
           구현하면 §4.6 위반이다.
    """
    hist = integrate(k.MISSION_PROFILE, MTOW, m_mot, E_batt, dv.n_ser,
                     pmap, aer, air, until_depleted=True)
    return AchievedRangeOut(
        R_dash=hist.x[-1],      # [스텁] 커널이 스텁이라 값도 가짜다  ← EC C2
        t_mission=hist.t[-1],   # [스텁]
    )
