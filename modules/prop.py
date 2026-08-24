"""PROP — 프롭 맵 · 작동점 · 모터 사이징 · 평가.  ICD0-008 §5.1

함수 다섯 개 중 solve_point 이 나머지가 공유하는 심장이다 (§4.6 커널 공유).
셀 개로전압 U_ocv(SOC) 도 전기 계통이라 여기 둔다 (§5 ▶부품 물성은 어디에).

모듈 간 직접 호출의 **유일한 예외**가 여기다 — size_motor 가 THRM 을 직접 부른다.
모터 크기 가정 → 동손 → 온도 → 판정 → 크기 조정이 한 이분법 안쪽이라
런처가 중간에 끼어들 수 없다 (§5 ▶모듈 간 직접 호출).

[스텁] P3(build_map·motor_elec·solve_point)·P4(size_motor) 에서 구현한다.
       evaluate 는 P3 이후. 반환값은 자리표시이며 물리적 근거가 없다.
"""
import math

import constants as k
from interfaces import (DesignVars, HullOut, AtmOut, AeroOut, PropMapOut,
                        MotorElecOut, SolvePointOut, SizeMotorOut, EvaluateOut)
from modules import thrm     # ← 유일하게 허용된 모듈 간 직접 호출 (§5)


# ══════════════════════════════════════════════════════════════════════════
# 부품 물성 — 셀 개로전압
# ══════════════════════════════════════════════════════════════════════════
def U_ocv(SOC: float) -> float:
    """셀당 개로전압 [V] — 3점 선형 (만충·공칭·종지).

    회귀가 아니라 상수 3개를 잇는 정의식이라 스텁이 아니다.
    상수 자체는 constants.py 에서 TBD 로 표시돼 있다.
    """
    SOC = min(max(SOC, 0.0), 1.0)
    if SOC >= 0.5:
        return k.U_cell_nom + (k.U_cell_full - k.U_cell_nom) * (SOC - 0.5) / 0.5
    return k.U_cell_cut + (k.U_cell_nom - k.U_cell_cut) * SOC / 0.5


def R_pack(E_batt: float, n_ser: int) -> float:
    """팩 내부저항 [Ω] — R = k_Rpack · n_ser / cap  (§3.2)."""
    cap_Ah = E_batt / max(U_ocv(1.0) * n_ser, 1e-9)     # Wh → Ah 환산
    return k.k_Rpack * n_ser / max(cap_Ah, 1e-9)


def U_eval(E_batt: float, n_ser: int, I_dash: float) -> float:
    """평가 전압 [V] — 방전 말기 최악 상태 (§4.5).

    U_eval = U_cell(1−DoD)·n_ser − I_dash·R_pack
    """
    return U_ocv(1.0 - k.DoD) * n_ser - I_dash * R_pack(E_batt, n_ser)


# ══════════════════════════════════════════════════════════════════════════
# ⓪ 프롭 성능 맵
# ══════════════════════════════════════════════════════════════════════════
def build_map(dv: DesignVars, air: AtmOut) -> PropMapOut:
    """⓪ BEMT 로 CT(J)·CP(J) 테이블을 미리 풀어 둔다.

    무차원 계수는 프롭 기하만의 함수라 모터·무게와 무관하다 → 루프 밖.

    [스텁] 구현 예정 (P3):
        BEMT(블레이드 요소 + 운동량 이론)를 전진비 J 격자에 대해 풀어 테이블 저장
        앵커 보정은 J 의 함수로 감쇠시킨다 — ICD §8 A-5 [판단 사항]
    """
    # 프롭 질량은 정의식 m = k_mprop·d³ (§5.1). 계수만 TBD.
    m_prop = k.k_mprop * dv.d_prop ** 3

    def CT(J: float) -> float:
        return 0.0      # [스텁] 실제 추력계수 아님

    def CP(J: float) -> float:
        return 0.0      # [스텁] 실제 동력계수 아님

    # g1 — 피치속도(= pitch × 허용 최대 rpm)가 V_cr 이상인지.
    # [스텁] 허용 최대 rpm 이 아직 없다 (팁 마하 한계에서 나온다) → 판정 보류.
    V_pitch = 0.0       # [스텁]
    g1 = 0.0            # [스텁] 실제 판정 아님

    return PropMapOut(CT=CT, CP=CP, m_prop=m_prop, g1=g1, V_pitch=V_pitch)


# ══════════════════════════════════════════════════════════════════════════
# 내부 — DC 모터 모델
# ══════════════════════════════════════════════════════════════════════════
def motor_elec(tau: float, omega: float, kv: float,
               R_mot: float, I0: float, U_bus: float) -> MotorElecOut:
    """고전 DC 모터 모델 (§4.3). 상수 효율 가정 금지.

        K_t = 60/(2π·K_v) ,  I = τ/K_t + I0 ,  U = I·R_mot + ω/K_v ,  P_cu = I²·R_mot

    수식 자체는 확정이고, R_mot·I0 를 주는 회귀 계수가 TBD 다 (§8 A-2).
    """
    K_t = 60.0 / (2.0 * math.pi * kv)       # [N·m/A] — kv[rpm/V] 와의 환산 정의
    I = tau / K_t + I0
    U_req = I * R_mot + omega / (kv * 2.0 * math.pi / 60.0)
    return MotorElecOut(I=I, U_req=U_req, P_cu=I * I * R_mot)


def motor_regression(m_mot: float, kv: float) -> tuple:
    """모터 질량·kv → (R_mot [Ω], I0 [A]).  ICD §8 A-2 회귀.

    [스텁] 회귀 계수가 아직 없다. constants.py 의 TBD 계수를 그대로 쓴다 —
           형태(멱함수)까지 잠정이며 스펙표 3~5종을 모으면 바뀔 수 있다.
    """
    R_mot = k.a_R * m_mot ** k.b_R * kv ** k.c_R    # [스텁] 계수 TBD
    I0 = k.a_I0 * kv ** k.b_I0                      # [스텁] 계수 TBD
    return R_mot, I0


# ══════════════════════════════════════════════════════════════════════════
# 공용 — 작동점 풀이
# ══════════════════════════════════════════════════════════════════════════
def solve_point(V: float, MTOW: float, m_mot: float,
                pmap: PropMapOut, aer: AeroOut, air: AtmOut,
                U_bus: float, hover: bool = False) -> SolvePointOut:
    """공용 작동점 — 평형이 두 겹이다 (§5.1 PROP).

      1. 기체 트림   : 미지수 (T, θ) 에 대한 2×2 뉴턴 (§4.1)
                       T·cosθ + L(α)·sinθ = W
                       T·sinθ − D(α)·cosθ = 0,   α = f(θ)
      2. 파워트레인 : 프롭 요구 토크 = 모터 발생 토크가 되는 rpm 을 1차원 근찾기

    호버(θ≈90°)는 1번을 건너뛰고 추력지지로 푼다.
    kv 는 이 토크 평형의 **해**로 나온다 — 설계변수가 아니다.

    [스텁] P3 에서 구현한다.
    [결정 필요] 두 겹을 중첩으로 풀지 하나의 잔차 벡터로 동시에 풀지 ·
                뉴턴 초기값 규칙 · 수렴 판정 기준 · 실패 시 처리(infeasible 권장)
    """
    return SolvePointOut(
        T=0.0,                                  # [스텁]
        theta=math.pi / 2 if hover else 0.0,    # [스텁]
        rpm=0.0,                                # [스텁]
        I=0.0,                                  # [스텁]
        P=0.0,                                  # [스텁]
        kv=0.0,                                 # [스텁]
        P_cu=0.0,                               # [스텁]
        ok=True,                                # [스텁] 실제 수렴 판정 아님
    )


# ══════════════════════════════════════════════════════════════════════════
# ① 모터 사이징
# ══════════════════════════════════════════════════════════════════════════
def size_motor(MTOW: float, pmap: PropMapOut, aer: AeroOut, air: AtmOut,
               U_bus: float, k_mot: float) -> SizeMotorOut:
    """① 요구 → 모터 질량.  m_mot 에 대한 **결정론적 이분법**.

    후보 질량마다:
      · 회귀에서 R_mot, I0 조회
      · dash 작동점 풀이 → 연속 축동력 충족 여부와 나선 팁 마하 (g2)
      · 호버 작동점 풀이 → 동손 → thrm.motor_rise → T_peak ≤ T_limit (g3)
      · 두 조건을 동시에 만족하는 최소 m_mot 을 찾은 뒤 k_mot 을 곱함

    ⚠ 내부 순환(크기 → 저항 → 효율 → 요구동력 → 크기)을 직전 반복값으로 닫지 않는다.
       이분법만 순수성을 지킨다 (§5 ▶모듈 내부 순환).

    [스텁] P4 에서 구현한다.
    """
    return SizeMotorOut(
        m_mot=0.0,          # [스텁] 실제 모터 질량 아님
        I_dash=0.0,         # [스텁]
        g2=0.0,             # [스텁] 실제 팁 마하 판정 아님
        g3=0.0,             # [스텁] 실제 열 판정 아님
        active="stub",      # [스텁] cruise/hover 판별 미구현
        n_bisect=0,
    )


# ══════════════════════════════════════════════════════════════════════════
# ② 성능 평가
# ══════════════════════════════════════════════════════════════════════════
def evaluate(MTOW: float, m_mot: float, E_batt: float, n_ser: int,
             pmap: PropMapOut, aer: AeroOut, air: AtmOut,
             U_bus: float) -> EvaluateOut:
    """② 최고속도 실계산 + 소음.

    최고속도를 여유 변수에서 읽지 않는다 (§4.2). 확정된 파워트레인으로
    추력 = 항력 평형을 만족하는 V_max 를 V 에 대한 1차원 이분법으로 직접 푼다.

    [스텁] P3 이후 구현한다. SPL 앵커는 78 dB @ 3 m 에서 팁속도·추력으로 스케일링.
    """
    return EvaluateOut(
        margin_V=0.0,       # [스텁] ← EC C3 (가중치 33.7%) 가 지금 가짜라는 뜻
        SPL_hover=0.0,      # [스텁] ← EC C6
        kv=0.0,             # [스텁]
        P_hover=0.0,        # [스텁]
        V_max=0.0,          # [스텁]
    )
