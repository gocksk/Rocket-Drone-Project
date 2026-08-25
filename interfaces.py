"""모듈 간 인터페이스 정의 — ICD0-008 §2·§5.1 의 코드판.

[규칙] 이 파일은 통합 담당만 수정한다. 필드를 바꾸려면 ICD 변경 절차(§9)를 먼저 거친다.
필드 이름은 ICD의 기호를 글자 그대로 쓴다. 단위는 SI 기본단위 (에너지만 Wh 허용).

접두어 규약 (§4.7): W_[kg] · P_[W] · E_[Wh] · T_[N] · F_[N] · U_[V] · I_[A]
                    eta_[-] · n_[-] · k_[-] · Cost_[KRW]
"""
from dataclasses import dataclass, field
from typing import Callable, Optional, Any


# ══════════════════════════════════════════════════════════════════════════
# §2 설계 변수
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class DesignVars:
    """최적화가 바꿔가며 탐색하는 값들.

    §2.1 형상·프롭 (표 8행 = 스칼라 10개) + §2.2 여유 변수 2개.
    kv·연속출력·E_batt 는 **설계변수가 아니다** — 요구조건에서 계산되어 나온다.
    """
    # ── §2.1 형상 · 프롭 ──
    d_body: float        # 동체 지름 [m]
    lambda_body: float   # 동체 세장비 (길이/지름) [-]
    S_fin: float         # 꼬리핀 총면적 [m²]
    x_fin: float         # 꼬리핀 위치 (기수 기준) [m]
    AR_fin: float        # 꼬리핀 종횡비 [-]
    f_mount: float       # 포드 결합 위치 (스팬 방향) [-]  범위 0.4–1.0
    n_design: float      # 튼튼함 수준 (프린트 인필·벽두께) [-]  범위 4–10  ← EC C4
    d_prop: float        # 프롭 지름 [m]  범위 0.10–0.18
    pd_prop: float       # 프롭 피치비 [-]  하한: 피치속도 ≥ 1.05·V_cr
    n_ser: int           # 배터리 셀 수 (이산) — 4S / 6S / 8S
    # ── §2.2 여유 변수 ──
    k_E: float           # 배터리를 요구 최소치의 몇 배로 실을까 [-]  범위 1.0–1.5
    k_mot: float         # 모터를 요구 최소치의 몇 배로 키울까 [-]  범위 1.0–1.4


# ══════════════════════════════════════════════════════════════════════════
# 공통 — 질량 항목
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class MassItem:
    """질량 분해표의 한 줄. 위치가 있어야 무게중심·관성이 나온다."""
    name: str
    m: float             # 질량 [kg]
    x: float             # 축방향 위치 (기수 기준) [m]
    r: float             # 동체축에서의 반경거리 [m] — J_xx 에 필수


# ══════════════════════════════════════════════════════════════════════════
# ATM — run(h)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class AtmOut:
    rho: float           # 공기밀도 [kg/m³]  → AERO, PROP, THRM
    T: float             # 온도 [K]          → THRM
    a_snd: float         # 음속 [m/s]        → PROP (팁 마하)
    nu: float            # 동점성계수 [m²/s] → AERO (Re)
    p: float             # 압력 [Pa]         — 진단
    mu: float            # 점성 [Pa·s]       — 진단


# ══════════════════════════════════════════════════════════════════════════
# GEOM — hull(dv) / layout(dims) / check_fit(…)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class HullOut:
    S_ref: float         # 동체 전면적 [m²] — 모든 항력계수의 기준면적
    S_wet: float         # 젖음면적 합 [m²]
    l_body: float        # 전장 [m]  ← EC C8
    r_body: float        # 동체 반경 [m]
    # 핀 기하 (Barrowman · STRC 용)
    b_fin: float         # 핀 스팬 (편측, 노출부) [m]
    c_root: float        # 루트 코드 [m]
    c_tip: float         # 팁 코드 [m]
    t_fin: float         # 핀 두께 [m]
    x_t: float           # 앞전 스윕 오프셋 (루트→팁) [m]
    x_fin: float         # 핀 루트 앞전 위치 (기수 기준) [m]
    # 파생 (진단·배치용)
    l_nose: float        # 노즈 길이 [m]
    l_cyl: float         # 원통부 길이 [m]
    # 젖음면적 분해 — ICD §5.1 출력 목록에는 S_wet 하나뿐이나, AERO 의 성분별
    # 항력 빌드업이 동체분과 핀분을 따로 쓴다. 같은 식을 두 모듈에 두지 않으려고
    # 여기서 나눠 낸다. [확정 필요]
    S_wet_body: float    # 노즈 + 원통 [m²]
    S_wet_fin: float     # 핀 양면 [m²]


@dataclass
class LayoutOut:
    x_parts: dict        # {부품명: 중심 x [m]} — 기수 기준
    arm_rotor: float     # 모터 암 길이 [m] → STAB, check_fit


@dataclass
class FitOut:
    g6: float            # 내장 여유 = 가용 길이 − 부품 점유 길이 (양수 합격)
    g7: float            # 클리어런스 = arm_rotor − d_prop/2 − r_body − 안전여유


# ══════════════════════════════════════════════════════════════════════════
# AERO — run(dv, hull, atm)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class AeroOut:
    # (V[m/s], α[rad], pod=(d_pod,l_pod)|None) → 항력 [N]
    F_drag: Callable[..., float]
    # (V[m/s], α[rad], pod) → 양력계수 [-] (S_ref 기준)
    # ICD §5.1 은 CL(α) 지만 축력항이 V 에 의존해 V 를 받도록 넓혔다 [로컬 개정]
    CL: Callable[..., float]
    CN_alpha: float                           # 법선력 기울기 [1/rad] → STAB
    x_cp: float                               # 압력중심 (기수 기준) [m] → STAB
    # ↓ ICD §5.1 AERO 출력 목록에는 없으나 STRC 입력이 요구하는 값 [확정 필요]
    CN_alpha_fin: float                       # 핀 성분 법선력 기울기 [1/rad]
    q_cr: float                               # 순항 동압 [Pa]
    CD0_cr: float                             # 진단: 순항 총 항력계수
    S_ref: float                              # 기준면적 [m²] — CL 을 힘으로 바꿀 때 필요.
                                              # hull 출력의 사본이다 [로컬 개정]


# ══════════════════════════════════════════════════════════════════════════
# PROP — build_map / motor_elec / solve_point / size_motor / evaluate
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class PropMapOut:
    CT: Callable[[float], float]   # J → 추력계수 [-]
    CP: Callable[[float], float]   # J → 동력계수 [-]
    m_prop: float                  # 프롭 1기 질량 [kg] → WGHT, COST
    g1: float                      # 순항 성립 = 피치속도 − V_cr (양수 합격)
    V_pitch: float                 # 진단: 피치속도 [m/s]
    d_prop: float                  # 프롭 지름 [m] — solve_point 가 J 를 만들 때 필요
                                   # [로컬 개정] ICD 출력 목록에는 없다


@dataclass
class MotorElecOut:
    I: float             # 전류 [A] → MISS (SOC 감소)
    U_req: float         # 요구 전압 [V] — 버스 전압과 비교해 성능 한계 판정
    P_cu: float          # 동손 I²·R_mot [W] → THRM


@dataclass
class SolvePointOut:
    T: float             # 총 요구추력 [N]
    theta: float         # 자세각 [rad] (호버 ≈ π/2)
    rpm: float           # 로터 회전수 [rpm]
    I: float             # 팩 전류 [A]
    P: float             # 소요 전력 [W]
    kv: float            # 토크 평형의 해로 나오는 kv [rpm/V] — 설계변수 아님
    P_cu: float          # 동손 [W] → THRM (**모터 1기당**)
    P_shaft: float       # 프롭 축동력 [W] (**로터 1기당**) — 연속 비출력 판정용
    U_req: float         # 모터 요구 전압 [V] — [로컬 개정 §11-16] kv 를 고정하고
                         # 부분 스로틀로 평가할 때 스로틀 = U_req/U_bus 가 된다
    ok: bool             # 수렴 여부 — False 면 해당 설계점 infeasible


@dataclass
class SizeMotorOut:
    m_mot: float         # 모터 1기 질량 [kg] → WGHT, COST
    I_dash: float        # dash 전류 [A] → MISS, U_eval 갱신
    g2: float            # 팁 마하 여유 = M_tip_max − M_tip (양수 합격)
    g3: float            # 열 한계 여유 = T_limit − T_peak (양수 합격)
    active: str          # 활성조건: "cruise" | "hover" — 모터 크기를 정한 쪽
    n_bisect: int        # 진단: 이분법 반복 횟수
    kv: float            # dash 점이 정한 kv [rpm/V] — 호버는 이 kv 로 부분 스로틀 평가
    T_peak: float        # 호버 종료 권선 온도 [°C] — 진단
    thr_hover: float     # 호버 스로틀 [-] — 진단. 1.0 을 넘으면 호버 자체가 불가
    P_shaft_dash: float  # dash 축동력 [W] — 진단 (연속 비출력 판정의 좌변)


@dataclass
class EvaluateOut:
    margin_V: float      # V_max / V_cr  ← EC C3
    SPL_hover: float     # 호버 소음 [dB] ← EC C6
    kv: float            # 요구 스펙 시트용 파생값 [rpm/V]
    P_hover: float       # 호버 동력 [W] — 진단
    V_max: float         # 최고속도 [m/s] — 진단


# ══════════════════════════════════════════════════════════════════════════
# THRM — motor_rise(…)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class ThrmOut:
    T_cruise_ss: float   # 순항 정상상태 온도 [°C] — 호버 계산의 초기값
    T_peak: float        # 호버 종료 시점 권선 온도 [°C] → PROP.size_motor
    T_hot: float         # 두 구간 중 **더 뜨거운** 쪽 [°C]  [로컬 개정 §11-17]
    hot_at: str          # 그게 어느 구간인가: "cruise" | "hover"  → 활성조건
    margin_T: float      # 열 여유 = T_limit − T_hot [K]
                         # ICD §5.1 은 T_peak 기준으로 적었으나, 이 기체는 순항이
                         # 더 뜨거울 수 있어 최대값 기준으로 바꿨다 (§11-17)


# ══════════════════════════════════════════════════════════════════════════
# MISS — integrate / required_energy / achieved_range
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class MissHistory:
    t: list              # 시각 [s]
    x: list              # 누적 거리 [m]
    E: list              # 누적 소비 에너지 [Wh]
    SOC: list            # 잔량 비율 [-]
    U_bus: list          # 버스 전압 [V]
    I_max: float         # 이력 중 최대 전류 [A]
    depleted: bool       # DoD 한계 도달 여부
    sustain_at: float    # 팩 전압이 모자라 추력을 못 버틴 첫 시각 [s]. 없으면 -1
                         # [로컬 개정 §11-22] 에너지 잔차와 분리해야 이분법이 단조다


@dataclass
class RequiredEnergyOut:
    E_batt: float        # 배터리 용량 [Wh] → GEOM(치수), COST
    m_batt: float        # 배터리 질량 [kg] → WGHT
    m_pack: float        # 팩 부자재 질량 [kg] → WGHT
    active: str          # 활성조건: "energy" | "power"
    n_bisect: int        # 진단: 이분법 반복 횟수
    E_energy: float      # 거리 요구가 정한 용량 [Wh] — 진단
    E_power: float       # 전류 요구가 정한 용량 [Wh] — 진단
    I_max: float         # 임무 중 최대 팩 전류 [A] — 진단, COST 의 ESC 정격


@dataclass
class AchievedRangeOut:
    R_dash: float        # 달성 항속거리 [m] ← EC C2
    t_mission: float     # 총 임무 시간 [s] — 진단


# ══════════════════════════════════════════════════════════════════════════
# STRC — run(MTOW, …)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class StrcOut:
    W_str: float         # 구조 무게 [kg] → WGHT
    m_print: float       # 프린트 재료 무게 [kg] → COST
    g5: float            # 응력 여유 (양수 합격)
    breakdown_str: list  # [MassItem] — WGHT 의 x_cg·J 용


# ══════════════════════════════════════════════════════════════════════════
# WGHT — converge(m_fixed, resp_of) / mass_props(…)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class WghtOut:
    """① 사이징 루프의 결과. 질량 특성(x_cg·J)은 배치(②) 뒤라 mass_props 가 낸다."""
    MTOW: float          # 최대이륙중량 [kg] ← EC C1
    g4: float            # 사이징 수렴 여부 (양수 합격)
    status: str          # converged / diverged_structural / diverged_numerical
                         #  / limit_cycle / max_iter
    S_hat: Optional[float]   # 성장계수 Ŝ — 진단. 추정 불가 시 None
    err: Optional[float]     # 반환 MTOW 의 오차 상한 [kg]. Ŝ ≥ 1 이면 None
    n_iter: int
    payload: Any         # 수렴 시점의 resp_of payload (불투명 객체)
    history: list        # [(MTOW, raw, resid)] — 진단·Ŝ 분해용


@dataclass
class MassProps:
    """② 배치 확정 후 1회 — 무게중심·관성·질량 분해표."""
    x_cg: float          # 무게중심 (기수 기준) [m] → STAB
    J_xx: float          # 3축 관성 [kg·m²] → STAB
    J_yy: float
    J_zz: float
    breakdown: list      # [MassItem] 전체. Σ = MTOW 항등


# ══════════════════════════════════════════════════════════════════════════
# STAB — run(…)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class StabOut:
    SM: float            # 정적 안정여유 [cal] — 진단
    alpha_max: float     # 조종 각가속도 [rad/s²] ← EC C5
    g8: float            # 직진 안정 = SM − SM_min (양수 합격)
    g9: float            # 천이 조종 = M_ctrl/M_dist − k_ctrl (양수 합격)
    M_dist: float        # 외란 모멘트 [N·m] — 상수가 아니라 계산값 (§4.4)
    M_ctrl: float        # 조종 모멘트 [N·m]


# ══════════════════════════════════════════════════════════════════════════
# COST — run(…)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class CostOut:
    Cost_acq: float      # 취득비 [KRW] ← EC C7
    breakdown: dict      # {항목: KRW}


# ══════════════════════════════════════════════════════════════════════════
# ⓪ 전처리 묶음 — main.preprocess(dv) 의 반환. 검증 도구도 같은 것을 부른다 (§8 C-1)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class PreOut:
    atm: AtmOut
    hull: HullOut
    aero: AeroOut
    pmap: PropMapOut


# ══════════════════════════════════════════════════════════════════════════
# ① 응답 질량 payload — 루프가 들여다보지 않는 불투명 객체 (§8 C-3)
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class RespPayload:
    m_mot: float             # 모터 1기 질량 [kg]
    m_batt: float
    m_pack: float
    E_batt: float            # [Wh]
    W_str: float
    smot: SizeMotorOut
    reqE: RequiredEnergyOut
    strc: StrcOut


# ══════════════════════════════════════════════════════════════════════════
# 최종 성적표
# ══════════════════════════════════════════════════════════════════════════
# 탈락 사유 코드 — DOE 로그의 1급 데이터다 (§5 ▶수렴·발산 분류).
# "설정 탓"과 "물리 탓"을 한 문자열로 뭉개면 정상 설계점이 조용히 후보에서 빠진다.
FAIL_NONE = ""
FAIL_GEOM = "geom_infeasible"       # ⓪ GEOM.hull — 형상 불성립 (ICD 등재 요청)
FAIL_G1 = "g1_pitch_speed"          # ⓪ PROP.build_map — 순항 미성립
FAIL_G2 = "g2_tip_mach"             # ① PROP.size_motor — 팁 마하 초과
FAIL_G3 = "g3_thermal"              # ① PROP.size_motor(←THRM) — 열 한계 초과
FAIL_G4_STRUCTURAL = "g4_diverged_structural"   # 무게 스노우볼 — 성립 불가 설계점
FAIL_G4_NUMERICAL = "g4_diverged_numerical"     # beta 선택 문제 — 재시도 가능
FAIL_G4_MAXITER = "g4_max_iter"                 # 반복 소진 — 잔여 오차 동반 보고
FAIL_TRIM = "trim_no_solution"      # 트림 연립 수렴 실패 (§4.1)


@dataclass
class Result:
    feasible: bool = True
    fail_code: str = FAIL_NONE     # 기계 판독용 사유 코드 — DOE 로그의 1급 데이터
    fail_stage: str = ""           # 사람용: 어느 구획에서 떨어졌나
    g: dict = field(default_factory=dict)     # {"g1": 값, ...} 음수면 불합격
    ec: dict = field(default_factory=dict)    # {"C1": 값, ...}
    diag: dict = field(default_factory=dict)  # 진단 변수 (활성조건·Ŝ·반복 횟수 …)
    pre: PreOut = None
    wght: WghtOut = None
    mass: MassProps = None
    layout: LayoutOut = None
    fit: FitOut = None
    eval: EvaluateOut = None
    rng: AchievedRangeOut = None
    stab: StabOut = None
    cost: CostOut = None
