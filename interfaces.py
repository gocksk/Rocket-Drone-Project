"""모듈 간 인터페이스 정의 = ICD0-007 §5의 코드판.

[규칙] 이 파일은 통합 담당만 수정한다.
여기 필드를 바꾸고 싶으면 ICD 변경 절차(§9)를 먼저 거친다.
필드 이름은 ICD·가이드라인과 글자 그대로 일치시킨다.
"""
from dataclasses import dataclass, field
from typing import Callable


# ───────── 설계변수 14개 (ICD §2) ─────────
@dataclass
class DesignVars:
    # 형상 변수 7
    d_body: float        # 동체 지름 [m]
    lambda_body: float   # 동체 세장비 [-]
    S_fin: float         # 핀 총면적 [m^2]
    x_fin: float         # 핀 루트 앞전 위치 (기수 기준) [m]
    AR_fin: float        # 핀 가로세로비 [-]
    f_mount: float       # 포드의 스팬 방향 결합 위치 (루트0·팁1) [-]
    n_design: float      # 설계 하중배수 [-]  ← EC C4
    # 부품 스펙 변수 7
    kv_mot: float        # 모터 kv [rpm/V]
    d_stat: float        # 스테이터 지름 [m]
    h_stat: float        # 스테이터 높이 [m]
    d_prop: float        # 프롭 지름 [m]
    pd_prop: float       # 프롭 피치비 [-]
    E_batt: float        # 배터리 에너지 [Wh]
    n_ser: int           # 배터리 직렬 수 [-]


# ───────── 질량 항목 (breakdown 공통 형식) ─────────
@dataclass
class MassItem:
    name: str
    m: float             # 질량 [kg]
    x: float             # 축방향 위치 (기수 기준) [m]
    r: float             # 동체축에서의 반경거리 [m] — J_xx 계산에 필수


# ───────── GEOM 출력 (GEOM 가이드라인 §8) ─────────
@dataclass
class GeomOut:
    l_body: float        # 전장 [m] ← EC C8
    l_nose: float
    l_cyl: float
    r_body: float
    S_ref: float         # 기준면적 = 동체 최대 단면적 [m^2]
    S_base: float
    S_wet: dict          # {"nose","cyl","fin","pod"} 성분별 [m^2]
    A_front_pod: float   # 포드 전면 투영 합 [m^2]
    N_junc: int          # 접합부 수
    # 핀 제원 (Barrowman·STRC용)
    b_1: float; c_r: float; c_t: float; x_t: float; t_fin: float
    x_fin: float
    # 포드·프롭 배치
    d_pod: float; l_pod: float
    x_pod: float; x_prop: float
    arm_rotor: float     # 모멘트 암 (파생값) [m]
    root_arm: float      # 핀 뿌리 모멘트 암 = f_mount*b_1 [m]
    # 내부 공간·부품 배치
    t_wall: float; d_int: float
    x_parts: dict        # {부품명: 중심 x [m]}
    # 합격
    g9: float
    g_clear: float       # 클리어런스 (별도 g 승격 여부는 미결)


# ───────── AERO 출력 (AERO 가이드라인 §8) ─────────
@dataclass
class AeroOut:
    F_drag: Callable[[float], float]     # V[m/s] → 항력[N]
    CN_alpha: float                      # 법선력 기울기 합 [1/rad]
    x_cp: float                          # 압력중심 (기수 기준) [m]
    CN_alpha_fin: float                  # 핀 성분만 (STRC 하중용)
    q_cr: float                          # 순항 동압 [Pa]
    C_N: Callable[[float], float]        # α[rad] → 고받음각 법선력계수
    x_cp_alpha: Callable[[float], float] # α[rad] → 압력중심 [m]
    x_cp_cross: float                    # 평면형 도심 [m]
    CD0_cr: float                        # 진단: 순항 총 항력계수


# ───────── PROP-A 출력 (PROP-A 가이드라인 §9) ─────────
@dataclass
class PropAOut:
    # 성능표 조회: (V[m/s], T_req_1로터[N], U[V]) → (P_elec[W], I_mot[A], Q_prop[N·m])
    query: Callable[[float, float, float], tuple]
    # 추력가용: (V[m/s], U[V]) → 로터 4개 합계 [N]
    T_avail: Callable[[float, float], float]
    # 배터리 전압: (SOC[-], I_pack[A]) → 팩 전압 [V]
    U_pack: Callable[[float, float], float]
    U_eval: float        # 방전 말기 최악 전압 [V]
    m_propsys: float     # 추진계 질량 (배선 포함) [kg]
    I_max: float         # 전개 최대 전류 [A]
    g4: float            # 방전 한계
    g5: float            # 팁 마하


# ───────── STRC 출력 (STRC 가이드라인 §9) ─────────
@dataclass
class StrcOut:
    m_str: float                       # 구조 질량 합 [kg]
    m_print: float                     # 프린트 재료량 [kg]
    m_fill: float                      # 핀 속 채움 질량 (MTOW 의존 항) [kg]
    w_fill: float                      # 핀 속 채움 폭 [m]
    breakdown_str: list                # [MassItem] — WGHT의 x_cg·J용
    g10: float                         # 핀 뿌리 강도 (수렴 후 판정)


# ───────── WGHT 출력 (WGHT 가이드라인 §7) ─────────
@dataclass
class WghtOut:
    MTOW: float          # [kg] ← EC C1
    x_cg: float
    J_xx: float; J_yy: float; J_zz: float
    breakdown: list      # [MassItem] 전체
    converged: bool
    n_iter: int
    strc: StrcOut        # 수렴 시점의 구조 결과 (g10 포함)
    # ↓ 수렴 진단 (WGHT §3). 기본값이 있어 기존 호출부는 그대로 동작한다.
    status: str = "converged"   # converged / diverged_structural / diverged_numerical
                                #  / limit_cycle / max_iter
    S_hat: float = None         # 수렴점 근방의 dm_str/dMTOW. 1에 가까울수록 되먹임이 강하다
    err: float = None           # 반환 MTOW의 오차 상한 [kg]. Ŝ>=1이면 None


# ───────── PROP-B 출력 (PROP-B 가이드라인 §8) ─────────
@dataclass
class PropBOut:
    V_max: float
    margin_V: float      # ← EC C3 (1위)
    P_hover: float       # [W]
    n_hover: float       # [rev/s]
    dT_max: float        # 차동추력 여유 (로터 1개) [N]
    dQ_max: float        # 차동토크 여유 [N·m]
    SPL_hover: float     # ← EC C6
    theta_req_cr: float  # 진단: 순항 자세각 [rad]
    g1: float; g2: float


# ───────── MISS 출력 (MISS 가이드라인 §6) ─────────
@dataclass
class MissOut:
    R_dash: float        # ← EC C2 [m]
    E_req: float         # 임무 총 에너지 [J]
    t_mission: float     # [s]
    seg_energy: dict     # 구간별 에너지 [J]
    g8: float


# ───────── STAB 출력 (STAB 가이드라인 §8) ─────────
@dataclass
class StabOut:
    SM: float            # [cal]
    r_ctrl: float
    M_avail_pitch: float # [N·m]
    M_avail_roll: float
    alpha_max: float     # ← EC C5 [rad/s^2]
    g6: float; g7: float


# ───────── COST 출력 (COST 가이드라인 §8) ─────────
@dataclass
class CostOut:
    Cost_acq: float      # ← EC C7 [KRW]
    Cost_op: float
    breakdown: dict


# ───────── 최종 성적표 ─────────
@dataclass
class Result:
    feasible: bool
    fail_stage: str = ""           # 조기 탈락 시 어디서 떨어졌나
    g: dict = field(default_factory=dict)   # {"g1": 값, ...} 음수면 불합격
    ec: dict = field(default_factory=dict)  # {"C1": MTOW, ...}
    geom: GeomOut = None
    aero: AeroOut = None
    propa: PropAOut = None
    wght: WghtOut = None
    propb: PropBOut = None
    miss: MissOut = None
    stab: StabOut = None
    cost: CostOut = None
