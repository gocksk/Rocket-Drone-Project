"""MISS — 임무 적분 · 배터리 사이징 · 달성 거리.  ICD0-008 §5.1

적분기는 **하나**다. required_energy(사이징용)와 achieved_range(평가용)는
같은 커널 integrate 의 다른 진입점일 뿐이다 (§4.6 이중 구현 금지).

  required_energy : 요구 거리를 고정하고 **E_batt** 를 이분법으로 찾는다
  achieved_range  : E_batt 를 고정하고 **거리** 를 이분법으로 찾는다

둘 다 "임무를 마쳤을 때 남는 에너지 = 0" 이라는 같은 잔차를 쓴다. 그래서
k_E=1.0 이면 achieved_range 가 R_dash_min 을 그대로 복원한다 — 이게 커널을
공유하고 있다는 증거다.

⚠ 고정 스텝 필수 — 적응 스텝은 잔차에 인공 노이즈를 만들어 WGHT 의
   resid_floor·양자화 판정 논리를 흐린다.

[구조 완료 — 계수 미확정]
`R_dash_min` `dt_miss` `e_spec` `c_rate_max` `k_Rpack` `k_trans` 세그먼트 시간이
전부 TBD 다. 반응 방향만 믿을 수 있다.
"""
import math

import constants as k
from interfaces import (DesignVars, AtmOut, AeroOut, PropMapOut,
                        MissHistory, RequiredEnergyOut, AchievedRangeOut)
from modules import prop


def segments(R_dash: float) -> list:
    """요구 임무 프로파일 — dash 세그먼트의 소요 시간을 **거리에서** 만든다.

    constants.MISSION_PROFILE 의 dash 항목은 t=None 이다. 사이징을 정하는 것은
    시간이 아니라 요구 거리(R_dash_min)이므로 여기서 t = R_dash/V 로 채운다.
    """
    out = []
    for name, mode, t, V in k.MISSION_PROFILE:
        if t is None:
            t = R_dash / max(V, 1e-9)
        out.append((name, mode, t, V))
    return out


def integrate(profile: list, MTOW: float, m_mot: float, kv: float, E_batt: float,
              dv: DesignVars, pmap: PropMapOut, aer: AeroOut,
              air: AtmOut, SOC_0: float = 1.0, pod=None) -> MissHistory:
    """공용 커널 — 고정 스텝 dt_miss 전진. 매 스텝마다:

        prop.solve_point 호출 → P → I → SOC 감소 → U_ocv(SOC)로 버스 전압 갱신 → 거리 누적

    모터는 이미 확정돼 있으므로 solve_point 를 **평가 모드**(kv 고정)로 부른다.
    그러면 작동점이 추력으로 정해지고 U_req·I 가 버스 전압과 무관해져, 팩 전압 강하를
    2차식으로 정확히 닫을 수 있다:

        U_bus = U_ocv(SOC)·n_ser − I_pack·R_pack ,  I_pack = P/U_bus
        →  U_bus² − U_oc·U_bus + P·R_pack = 0

    직전 스텝의 전류를 재사용하지 않으므로 시간 지연 오차가 없다.

    프로파일은 SOC 가 바닥나도 **끝까지** 돈다. 중간에 끊지 않는 이유는 이 커널의
    쓰임이 '남는 에너지'라는 연속 잔차를 이분법에 주는 것이기 때문이다 —
    끊으면 잔차가 계단이 되어 이분법이 깨진다. 고갈 여부는 depleted 로 알린다.
    """
    R_pk = prop.R_pack(E_batt, dv.n_ser)
    E_usable = E_batt * k.DoD

    t = x = E_used = 0.0
    I_max = 0.0
    sustain_at = -1.0
    ts, xs, Es, socs, Us = [0.0], [0.0], [0.0], [SOC_0], []

    for name, mode, t_seg, V in profile:
        # 작동점은 세그먼트 안에서 **불변**이다. kv 가 고정된 평가 모드에서는 추력이
        # rpm 을 정하고 rpm 이 (τ, ω) 를 정하므로 U_req·I·P 가 버스 전압과 무관하다.
        # 버스 전압에 의존하는 것은 '그 전압으로 버틸 수 있는가' 하나뿐이라, 트림과
        # rpm 이분법을 스텝마다 다시 풀 이유가 없다. 세그먼트당 1회로 줄인다.
        # (전압 판정은 아래에서 스텝마다 실제 U_bus 로 따로 한다. 그래서 U_ref 는
        #  판정이 걸리지 않을 만큼 크게 준다 — 순수성에는 영향이 없다.)
        U_ref = 1.0e9
        sp = prop.solve_point(V, MTOW, m_mot, pmap, aer, air, U_ref,
                              hover=(mode == "hover"), kv=kv, pod=pod)
        if not sp.ok:
            # 트림이나 rpm 이 안 풀린다 — 임무가 성립하지 않는다
            return MissHistory(t=ts, x=xs, E=Es, SOC=socs, U_bus=Us,
                               I_max=I_max, depleted=True, sustain_at=t)
        # 천이는 가감속 때문에 정상비행보다 동력을 더 쓴다 (§3.1 k_trans)
        P = sp.P * (k.k_trans if mode == "trans" else 1.0)

        n_step = max(1, int(math.ceil(t_seg / k.dt_miss - 1e-12)))
        for i in range(n_step):
            dt_i = min(k.dt_miss, t_seg - i * k.dt_miss)
            if dt_i <= 0.0:
                break
            SOC = max(SOC_0 - E_used / max(E_batt, 1e-12), 0.0)
            U_oc = prop.U_ocv(SOC) * dv.n_ser

            # 팩 내부저항에 의한 전압 강하 — 2차식의 물리적 근(높은 전압).
            # 판별식이 음수면 팩의 최대전달전력(U_oc²/4R)을 넘은 것이다.
            disc = U_oc * U_oc - 4.0 * P * R_pk
            if disc > 0.0:
                U_bus = 0.5 * (U_oc + math.sqrt(disc))
            else:
                U_bus = 0.5 * U_oc                  # 최대전달전력점
            if (disc <= 0.0 or sp.U_req > U_bus) and sustain_at < 0.0:
                sustain_at = t                      # 못 버틴 첫 시각만 기록

            # ⚠ 여기서 멈추지 않는다. 이 커널의 쓰임은 '남는 에너지'라는 **연속 잔차**를
            #   이분법에 주는 것이다. 전압이 모자란 순간 끊으면 잔차가 계단이 되어
            #   이분법이 깨진다(달성거리가 탐색 상한에 붙는 증상). 지속 불가는
            #   sustain_at 으로 따로 알리고, 에너지 수지는 끝까지 센다.
            I_pack = P / max(U_bus, 1e-9)
            I_max = max(I_max, I_pack)
            # 셀에서 빠져나가는 전력은 ESC 입력 P 가 아니라 U_oc·I_pack 이다 —
            # 차액 I²·R_pack 이 팩 내부에서 열로 버려진다. 이 항이 있어야
            # "전압이 떨어지면 같은 추력에 전류가 커지는 말기 악화"(§5.1 MISS)가
            # 실제로 나타난다. 빼면 소비 전력이 구간 내내 상수라 적분이 스텝에
            # 완전히 무관해지고, 그건 물리가 아니라 모델의 공백이다.
            E_used += U_oc * I_pack * dt_i / 3600.0        # [Wh]
            x += V * dt_i
            t += dt_i

            ts.append(t); xs.append(x); Es.append(E_used)
            socs.append(max(SOC_0 - E_used / max(E_batt, 1e-12), 0.0))
            Us.append(U_bus)

    return MissHistory(t=ts, x=xs, E=Es, SOC=socs, U_bus=Us, I_max=I_max,
                       depleted=(E_used >= E_usable), sustain_at=sustain_at)


def _left(E_batt: float, R_dash: float, MTOW, m_mot, kv, dv, pmap, aer, air, pod=None) -> tuple:
    """임무를 마치고 **남는 가용 에너지** [Wh] 와 이력. 이분법의 잔차다.

    양수면 여유가 있고 음수면 모자란다. E_batt 에 대해 증가, R_dash 에 대해 감소라
    두 이분법 모두 단조다.
    """
    h = integrate(segments(R_dash), MTOW, m_mot, kv, E_batt, dv, pmap, aer, air, pod=pod)
    return E_batt * k.DoD - h.E[-1], h


def _bisect_E(pred) -> float:
    """pred(E) 가 참이 되는 최소 E_batt — 결정론적 이분법. pred 는 E 에 단조여야 한다."""
    lo, hi = k.E_batt_lo, k.E_batt_hi
    if not pred(hi):
        return hi                       # 상한에서도 안 된다
    for _ in range(k.N_bisect_max):
        mid = 0.5 * (lo + hi)
        if pred(mid):
            hi = mid
        else:
            lo = mid
        if hi - lo < k.eps_bisect_rel * k.E_batt_hi:
            break
    return hi


def required_energy(MTOW: float, m_mot: float, kv: float, dv: DesignVars,
                    pmap: PropMapOut, aer: AeroOut, air: AtmOut,
                    pod=None) -> RequiredEnergyOut:
    """① 요구 임무 실적분 → 배터리 사이징.  E_batt 에 대한 **결정론적 이분법**.

      · 후보 용량마다 m_batt = E_batt/e_spec, R_pack = k_Rpack·n_ser/cap 로 닫고
        integrate 를 돌려 요구 거리(R_dash_min) 도달 시점의 SOC 여유를 본다
      · 여유가 0(DoD 한계)이 되는 최소 용량 → E_energy
      · 전류 요구에서 E_power = I_max / c_rate_max
      · E_batt = k_E × max(E_energy, E_power)

    ⚠ 내부 순환(E_batt → R_pack → 전압 → 전류 → 소비 에너지 → E_batt)을 직전
       반복값으로 닫지 않는다. 이분법만 순수성을 지킨다.

    MTOW 는 이 함수 안에서 **고정**이다 — 배터리가 무거워져 다시 에너지가 느는
    되먹임은 바깥 WGHT 루프의 몫이다. 그래서 여기서는 잔차가 E_batt 에 단조다.
    """
    def run(E):
        return _left(E, k.R_dash_min, MTOW, m_mot, kv, dv, pmap, aer, air, pod)

    # (1) 거리 요구 — 남는 가용 에너지가 0 이 되는 최소 용량
    E_energy = _bisect_E(lambda E: run(E)[0] >= 0.0)

    # (2) 지속 가능성 — 팩 전압 강하 때문에 요구 추력을 못 버티는 구간이 없어야 한다.
    #     ICD 는 C-rate 상한(E_power)만 적었는데, c_rate_max 와 k_Rpack 이 서로
    #     맞지 않으면 "규정상 허용되는데 물리적으로 불가능한" 팩이 나온다.
    #     실제로 지금 상수 조합(90C · k_Rpack=0.010)이 그렇다. [로컬 개정 §11-23]
    E_sustain = _bisect_E(lambda E: run(E)[1].sustain_at < 0.0)

    # (3) 전류 요구 — 연속 방전율 한계.
    #     ICD 는 E_power = I_max/c_rate_max 로 적었는데 이건 [Ah] 다.
    #     Wh 로 맞추려면 공칭 팩 전압을 곱해야 한다. [로컬 개정 §11-21]
    h = run(max(E_energy, E_sustain))[1]
    E_power = (h.I_max / k.c_rate_max) * k.U_cell_nom * dv.n_ser

    E_min = max(E_energy, E_power, E_sustain)
    active = ("energy" if E_min == E_energy else
              "power" if E_min == E_power else "sustain")
    E_batt = dv.k_E * E_min
    m_batt = E_batt / k.e_spec
    return RequiredEnergyOut(E_batt=E_batt, m_batt=m_batt,
                             m_pack=k.k_pack * m_batt, active=active,
                             n_bisect=k.N_bisect_max, E_energy=E_energy,
                             E_power=E_power, I_max=h.I_max)


def achieved_range(MTOW: float, m_mot: float, kv: float, E_batt: float,
                   dv: DesignVars, pmap: PropMapOut, aer: AeroOut,
                   air: AtmOut, pod=None) -> AchievedRangeOut:
    """② 확정 스펙으로 달성 거리 — **같은 커널**을 거리에 대해 이분법으로 돌린다.

    "SOC 소진까지 dash 를 계속한다" 를 그대로 구현하면 착륙 몫이 남지 않아
    required_energy 와 기준이 어긋난다(요구 임무는 착륙 호버를 포함한다).
    그래서 **천이·착륙까지 마치고 정확히 DoD 에 도달하는 dash 거리**를 찾는다.
    이 정의라야 k_E=1.0 에서 R_dash ≈ R_dash_min 이 성립한다.
    """
    lo, hi = 0.0, k.R_dash_cap
    if _left(E_batt, lo, MTOW, m_mot, kv, dv, pmap, aer, air, pod)[0] < 0.0:
        return AchievedRangeOut(R_dash=0.0, t_mission=0.0)   # 이착륙조차 못 한다
    if _left(E_batt, hi, MTOW, m_mot, kv, dv, pmap, aer, air, pod)[0] >= 0.0:
        return AchievedRangeOut(R_dash=hi, t_mission=0.0)    # 탐색 상한 초과
    for _ in range(k.N_bisect_max):
        mid = 0.5 * (lo + hi)
        if _left(E_batt, mid, MTOW, m_mot, kv, dv, pmap, aer, air, pod)[0] >= 0.0:
            lo = mid
        else:
            hi = mid
        if hi - lo < k.eps_bisect_rel * k.R_dash_cap:
            break
    R = lo
    h = _left(E_batt, R, MTOW, m_mot, kv, dv, pmap, aer, air, pod)[1]
    return AchievedRangeOut(R_dash=R, t_mission=h.t[-1])
