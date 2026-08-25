"""STRC — 프린트 구조 질량과 핀 루트 1차 구조 사이징.

**본문은 STRC 담당이 쓴 코드다.** 아래 계산 로직·함수 구조·변수명을 그대로 두고,
ICD0-008 인터페이스와의 차이만 이 파일 맨 아래 어댑터(·)에 가뒀다.
담당자가 ICD0-008 로 갱신하면 어댑터만 지우면 된다.

원본 대비 손댄 곳은 **네 군데뿐**이고 전부  로 표시돼 있다.
  1/4  핀 스팬 b_fin — ICD0-008 의 f_mount 때문에 "로터축 = 핀 끝" 전제가 안 선다
  2/4  q_cr · 핀 법선력 계수를 AERO 에서 받는다 (§5.1). 원본은 자체 계산했다
  3/4  반환을 dict 로 — ICD0-007 의 StrcOut(m_str·g10) 매핑을 run() 한 곳에 모은다
  4/5  쉘 도심을 젖음면적 가중으로 (docs §11-35 에서 잡은 버그)
  5/5  getattr 기본값 즉시평가 버그 — 원본이 무조건 dv.arm_rotor 를 건드린다

── 담당자 원본 문서 (그대로 옮김) ──────────────────────────────────
계산 범위
  1. 동체 쉘, 핀 4장, 모터 포드 4개의 기본 구조 질량
  2. 로터 추력과 고속 off-design 핀 공력에 의한 핀 루트 굽힘/전단
  3. 필요한 핀 루트 100% 인필 보강 폭과 추가 질량
  4. WGHT가 MTOW, x_cg, J_yy를 계산할 수 있도록 breakdown_str 반환

중요한 역할 분담
  - STRC는 m_str(ICD의 W_str), m_print, breakdown_str을 반환한다.
  - MTOW, x_cg, J_xx, J_yy, J_zz는 WGHT가 계산한다.
  - STRC 내부에는 MTOW 수렴 반복문을 두지 않는다.

현재 모델에서 제외하는 항목
  링프레임, 격벽, 접합부 상세, 동체 좌굴, 로터 반작용 토크,
  착륙 충격, 피로/진동, 상세 비틀림, FEM, 천이 시간 이력
───────────────────────────────────────────────────────────────────

[MTOW 반응] 쉘·핀·포드는 MTOW 무관이고, **핀 루트 보강(m_fill)만 MTOW 에 반응**한다
(하중 케이스의 T_lim = n_design·MTOW·g/N_rot). 하중이 실제로 들어가는 자리라
물리적으로 옳고, docs §11-33 이 지적한 "구조가 MTOW 에 안 반응하는 문제"를 원본이
이미 이 방식으로 풀고 있었다.

⚠ w_fill 이 line width 정수배로 **양자화**된다. ICD §5.1 STRC 가 경고한 계단형
  출력이 여기서 실제로 생기므로, MTOW 가 두 값 사이를 진동하면 발산이 아니라
  WGHT 의 limit_cycle 로 분류돼야 한다.

[구조 완료 — 계수 미확정] 슬라이서·재료 상수가 전부 TBD 다 (constants.py 참조).
"""

from __future__ import annotations

from math import ceil, hypot, pi, sqrt
from typing import Dict, List

import constants as k
from interfaces import DesignVars, HullOut, AeroOut, MassItem, StrcOut


_EPS = 1.0e-12


def _constant(name: str, default: float) -> float:
    """constants.py에 값이 없을 때 초기 기본값을 사용한다."""
    return float(getattr(k, name, default))


def _required_attr(obj, name: str) -> float:
    """필수 GEOM 필드를 읽고, 없으면 필요한 필드명을 분명하게 알린다."""
    if not hasattr(obj, name):
        raise AttributeError(f"GeomOut에 STRC 필수 필드 '{name}'가 없습니다.")
    return float(getattr(obj, name))


def _body_wet_area(geo: GeomOut) -> float:
    """동체만의 표면적을 읽는다.

    신규 필드 ``S_wet_body``를 우선 사용한다. 이전 GEOM 형식과 임시 호환을
    위해 ``S_wet`` 딕셔너리의 nose/cyl 값을 fallback으로 허용한다.
    """
    if hasattr(geo, "S_wet_body"):
        return float(geo.S_wet_body)

    if hasattr(geo, "S_wet") and isinstance(geo.S_wet, dict):
        return float(geo.S_wet.get("nose", 0.0) + geo.S_wet.get("cyl", 0.0))

    raise AttributeError(
        "GeomOut에 'S_wet_body'가 필요합니다. "
        "전체 S_wet가 아니라 동체만의 표면적이어야 합니다."
    )


def _print_settings(dv: DesignVars) -> Dict[str, float]:
    """n_design을 기본 인필과 핀 외피 설정으로 변환한다."""
    phi = _constant("phi_0", 0.15) + _constant("k_phi", 0.02) * (
        float(dv.n_design) - _constant("n_ref_load", 4.0)
    )
    k_dens = _constant("k_dens", 0.95)
    phi = min(max(phi, 0.0), k_dens - 1.0e-9)

    w_line = _constant("w_line", 0.00045)
    n_peri = _constant("n_peri", 3.0)

    return {
        "phi": phi,
        "w_line": w_line,
        "t_skin": n_peri * w_line,
        "k_dens": k_dens,
    }


def _fin_geometry(dv: DesignVars, geo: GeomOut) -> Dict[str, float]:
    """도면과 GEOM 출력으로 핀 구조 계산에 필요한 파생 형상을 만든다."""
    n_rot = int(_constant("N_rot", 4.0))
    r_body = float(getattr(geo, "r_body", float(dv.d_body) / 2.0))
    # [이식 5/5] 원본은 getattr(geo, "arm_rotor", float(dv.arm_rotor)) 였다.
    # 파이썬은 getattr 의 기본값을 **먼저** 평가하므로, geo 에 arm_rotor 가 있어도
    # dv.arm_rotor 를 먼저 건드려 터진다. ICD0-008 의 DesignVars 에는 arm_rotor 가
    # 없다(f_mount 로 유도한다). 지연 평가로 바꾼다 — 담당자에게 보고할 것.
    if hasattr(geo, "arm_rotor"):
        arm_rotor = float(geo.arm_rotor)
    else:
        arm_rotor = float(dv.arm_rotor)

    # [이식 1/3] 원본은 "로터축이 핀 끝에 있다"고 보고 b_fin = arm_rotor − r_body 로
    # 썼다. ICD0-008 에는 f_mount(포드의 스팬 방향 결합 위치)가 있어 로터가 핀 끝이
    # 아닐 수 있으므로, **핀 스팬**은 GEOM 이 낸 b_fin 을 그대로 쓴다.
    # 루트 모멘트 암은 로터가 실제로 붙은 위치여야 하므로 arm_rotor − r_body 다.
    b_fin = float(getattr(geo, "b_fin", arm_rotor - r_body))
    root_arm = arm_rotor - r_body
    if b_fin <= 0.0:
        raise ValueError(
            f"b_fin={b_fin:.6g} m입니다. arm_rotor는 d_body/2보다 커야 합니다."
        )

    c_r = _required_attr(geo, "c_r")
    c_t = _required_attr(geo, "c_t")
    t_fin = _required_attr(geo, "t_fin")
    if min(c_r, c_t, t_fin) <= 0.0:
        raise ValueError("c_r, c_t, t_fin은 모두 0보다 커야 합니다.")

    S_fin_1 = float(dv.S_fin) / n_rot
    if S_fin_1 <= 0.0:
        raise ValueError("S_fin / N_rot는 0보다 커야 합니다.")

    # 사다리꼴 핀의 면적중심을 공력 작용점의 스팬 방향 근사로 사용.
    s_cp_fin = b_fin * (c_r + 2.0 * c_t) / (3.0 * (c_r + c_t))

    return {
        "_q_cr": float(getattr(geo, "q_cr", 0.0)) or None,
        "_CN_alpha_fin_1": float(getattr(geo, "CN_alpha_fin_1", 0.0)) or None,
        "n_rot": float(n_rot),
        "r_body": r_body,
        "arm_rotor": arm_rotor,
        "b_fin": b_fin,
        "S_fin_1": S_fin_1,
        "c_r": c_r,
        "c_t": c_t,
        "t_fin": t_fin,
        "root_arm": root_arm,
        "s_cp_fin": s_cp_fin,
    }


def _cruise_dynamic_pressure() -> float:
    """300 km/h 구조 설계점의 동압을 STRC 내부에서 계산한다."""
    rho_air = _constant("rho_air", 1.225)
    V_cr = _constant("V_cr", 83.3)
    return 0.5 * rho_air * V_cr**2


def _fin_normal_slope(fin: Dict[str, float]) -> float:
    """핀 한 장의 normal-force 기울기 [1/rad]를 저차식으로 계산한다.

    핀 한 장을 독립된 유한날개로 보고 다음 근사를 사용한다.

        AR = b_fin**2 / S_fin_1
        CN_alpha_fin = 2*pi*AR / (2 + sqrt(4 + AR**2))

    동체 간섭, 후퇴각, 점성효과는 포함하지 않는다. 구조 설계용 초기 근사다.
    """
    aspect_ratio = fin["b_fin"] ** 2 / fin["S_fin_1"]
    return 2.0 * pi * aspect_ratio / (2.0 + sqrt(4.0 + aspect_ratio**2))


def _allowables() -> tuple[float, float]:
    """FDM 적층 저하와 재료 안전계수를 반영한 굽힘/전단 허용응력."""
    sigma_cat = _constant("sigma_cat", 45.0e6)
    tau_cat = _constant("tau_cat", sigma_cat / sqrt(3.0))
    sf = _constant("SF", 1.5)

    k_layer_b = _constant("k_layer_b", _constant("k_layer", 0.5))
    k_layer_s = _constant("k_layer_s", _constant("k_layer", 0.5))

    sigma_allow = k_layer_b * sigma_cat / sf
    tau_allow = k_layer_s * tau_cat / sf

    if sigma_allow <= 0.0 or tau_allow <= 0.0:
        raise ValueError("sigma_allow와 tau_allow는 0보다 커야 합니다.")

    return sigma_allow, tau_allow


def fixed_masses(dv: DesignVars, geo: GeomOut) -> Dict[str, float]:
    """MTOW와 무관한 동체 쉘, 기본 핀, 포드 쉘 질량을 계산한다.

    같은 설계 후보의 STRC-WGHT 반복에서는 한 번만 호출한 뒤 결과를 재사용한다.
    반환값은 실제 기체에 남는 장착 질량 [kg]이다.
    """
    rho = _constant("rho_mat", 1200.0)
    cfg = _print_settings(dv)
    fin = _fin_geometry(dv, geo)

    t_wall = _required_attr(geo, "t_wall")
    d_pod = _required_attr(geo, "d_pod")
    l_pod = _required_attr(geo, "l_pod")
    t_wall_pod = float(
        getattr(geo, "t_wall_pod", _constant("t_wall_pod", t_wall))
    )

    if min(t_wall, d_pod, l_pod, t_wall_pod) <= 0.0:
        raise ValueError("t_wall, d_pod, l_pod, t_wall_pod는 0보다 커야 합니다.")

    # 1) 동체 쉘
    m_shell = (
        rho
        * _body_wet_area(geo)
        * t_wall
        * _constant("k_sl_shell", 1.0)
    )

    # 2) 핀 4장: 외피 + 기본 인필
    V_fin_1 = _constant("k_sec", 1.0) * fin["S_fin_1"] * fin["t_fin"]
    V_skin_1 = min(2.0 * fin["S_fin_1"] * cfg["t_skin"], V_fin_1)
    V_core_1 = max(V_fin_1 - V_skin_1, 0.0)
    m_fin_1 = (
        rho
        * (V_skin_1 + cfg["phi"] * V_core_1)
        * _constant("k_sl_fin", 1.0)
    )
    m_fin = int(fin["n_rot"]) * m_fin_1

    # 3) 포드 4개: 얇은 원통 쉘 근사
    m_pod_1 = (
        rho
        * pi
        * d_pod
        * l_pod
        * t_wall_pod
        * _constant("k_sl_pod", 1.0)
    )
    m_pod = int(fin["n_rot"]) * m_pod_1

    return {
        "shell": m_shell,
        "fin": m_fin,
        "pod_shell": m_pod,
    }


def _load_cases(dv: DesignVars, fin: Dict[str, float], MTOW: float) -> List[Dict[str, float]]:
    """초기 핀 루트 하중 케이스 세 개를 만든다."""
    if MTOW <= 0.0:
        raise ValueError("MTOW는 0보다 커야 합니다.")

    g0 = _constant("g", 9.80665)
    T_lim = float(dv.n_design) * MTOW * g0 / int(fin["n_rot"])

    # [이식 2/3] ICD §5.1 은 STRC 입력에 q_cr 과 핀 법선력 계수를 **AERO 에서** 받으라고
    # 정한다. 원본은 둘 다 자체 계산했는데(파일 주석 "AERO 입력은 사용하지 않는다"),
    # 그러면 같은 물리량을 두 모듈이 따로 갖게 되어 §4.6 이 경계하는 발산이 생긴다.
    # 값이 넘어오면 그것을 쓰고, 없으면 원본 자체 계산으로 되돌아간다.
    q_cr = fin.get("_q_cr") or _cruise_dynamic_pressure()
    CN_alpha_fin = fin.get("_CN_alpha_fin_1") or _fin_normal_slope(fin)
    alpha_lim = _constant("alpha_lim", 5.0 * pi / 180.0)
    N_aero = q_cr * fin["S_fin_1"] * CN_alpha_fin * alpha_lim

    M_in = T_lim * fin["root_arm"]
    M_out = N_aero * fin["s_cp_fin"]

    return [
        {
            "name": "LC_THRUST",
            "V_in": T_lim,
            "V_out": 0.0,
            "M_in": M_in,
            "M_out": 0.0,
        },
        {
            "name": "LC_HIGH_SPEED_OFFDESIGN",
            "V_in": 0.0,
            "V_out": N_aero,
            "M_in": 0.0,
            "M_out": M_out,
        },
        {
            "name": "LC_BOUND",
            "V_in": T_lim,
            "V_out": N_aero,
            "M_in": M_in,
            "M_out": M_out,
        },
    ]


def _root_stress(case: Dict[str, float], width: float, t_fin: float) -> tuple[float, float]:
    """직사각형 등가단면의 조합 굽힘응력과 최대 전단응력."""
    width = max(width, _EPS)
    t_fin = max(t_fin, _EPS)

    sigma = (
        6.0 * abs(case["M_in"]) / (t_fin * width**2)
        + 6.0 * abs(case["M_out"]) / (width * t_fin**2)
    )

    V_resultant = hypot(case["V_in"], case["V_out"])
    tau = _constant("k_tau", 1.5) * V_resultant / (width * t_fin)
    return sigma, tau


def _required_width(
    case: Dict[str, float],
    t_fin: float,
    sigma_allow: float,
    tau_allow: float,
    w_min: float,
) -> float:
    """한 하중 케이스를 만족하는 최소 폭을 닫힌식으로 계산한다.

    굽힘식은 다음 꼴이다.

        sigma = A / w**2 + B / w

    이를 ``sigma <= sigma_allow``로 풀어 양의 근을 사용한다.
    """
    A = 6.0 * abs(case["M_in"]) / t_fin
    B = 6.0 * abs(case["M_out"]) / t_fin**2

    w_bend = (B + sqrt(B**2 + 4.0 * sigma_allow * A)) / (
        2.0 * sigma_allow
    )

    V_resultant = hypot(case["V_in"], case["V_out"])
    w_shear = (
        _constant("k_tau", 1.5)
        * V_resultant
        / (tau_allow * t_fin)
    )

    return max(w_min, w_bend, w_shear)


def _solve(
    dv: DesignVars,
    geo,
    MTOW: float,
    fixed: Dict[str, float] | None = None,
) -> Dict[str, object]:
    """현재 MTOW에서 구조 질량과 핀 루트 보강 폭을 계산한다.

    WGHT에서는 다음처럼 호출한다.

        strc_fixed = strc.fixed_masses(dv, geo)
        st = strc.run(dv, geo, MTOW, strc_fixed)
    """
    if fixed is None:
        fixed = fixed_masses(dv, geo)

    expected_keys = {"shell", "fin", "pod_shell"}
    if set(fixed) != expected_keys:
        raise ValueError(
            f"fixed_masses 키는 {sorted(expected_keys)}여야 합니다: {sorted(fixed)}"
        )

    fin = _fin_geometry(dv, geo)
    cfg = _print_settings(dv)
    sigma_allow, tau_allow = _allowables()
    cases = _load_cases(dv, fin, MTOW)

    w_min = max(2.0 * cfg["w_line"], _constant("w_fill_min", 0.001))
    w_req = max(
        _required_width(
            case=case,
            t_fin=fin["t_fin"],
            sigma_allow=sigma_allow,
            tau_allow=tau_allow,
            w_min=w_min,
        )
        for case in cases
    )

    # 실제 출력 가능한 line width의 정수배로 올림.
    w_fill = ceil(w_req / cfg["w_line"]) * cfg["w_line"]
    w_fill_max = _constant("k_w", 0.6) * fin["c_r"]

    # 결정된 폭으로 세 케이스를 다시 계산해 최악 여유를 구한다.
    sigmas: List[float] = []
    taus: List[float] = []
    for case in cases:
        sigma, tau = _root_stress(case, w_fill, fin["t_fin"])
        sigmas.append(sigma)
        taus.append(tau)

    sigma_max = max(sigmas)
    tau_max = max(taus)

    g_width = w_fill_max / max(w_fill, _EPS) - 1.0
    g_bend = sigma_allow / max(sigma_max, _EPS) - 1.0
    g_shear = tau_allow / max(tau_max, _EPS) - 1.0
    g10 = min(g_width, g_bend, g_shear)

    # 기본 핀에 이미 존재하는 phi 인필과 100% modifier의 차이만 추가한다.
    t_core = max(fin["t_fin"] - 2.0 * cfg["t_skin"], 0.0)
    V_fill_1 = (
        w_fill
        * t_core
        * fin["b_fin"]
        * _constant("k_taper", 0.6)
    )
    delta_phi = max(cfg["k_dens"] - cfg["phi"], 0.0)
    m_fill_1 = (
        _constant("rho_mat", 1200.0)
        * delta_phi
        * V_fill_1
        * _constant("k_sl_fill", 1.0)
    )
    m_fill = int(fin["n_rot"]) * m_fill_1

    # breakdown_str: 첫 항목은 WGHT가 길이 관성항에 사용하는 shell이어야 한다.
    l_body = float(getattr(geo, "l_body", float(dv.d_body) * float(dv.lambda_body)))
    # [이식 4/5] 원본은 쉘을 전장 중점에 뒀는데, 쉘 질량은 젖음면적을 따라가므로
    # 노즈분(전체의 36%)이 뒤로 밀린다. GEOM 이 낸 면적가중 도심을 쓴다 (docs §11-35).
    x_shell = float(getattr(geo, "x_wet_body", 0.5 * l_body))
    x_fin_cg = float(dv.x_fin) + 0.5 * fin["c_r"]
    x_pod = float(getattr(geo, "x_pod", x_fin_cg))

    r_fin = fin["r_body"] + _constant("k_r", 0.4) * fin["b_fin"]
    r_fill = fin["r_body"] + _constant("k_r_fill", 0.4) * fin["b_fin"]

    breakdown = [
        MassItem("shell", fixed["shell"], x_shell, fin["r_body"]),
        MassItem("fin", fixed["fin"], x_fin_cg, r_fin),
        MassItem("pod_shell", fixed["pod_shell"], x_pod, fin["arm_rotor"]),
        MassItem("root_fill", m_fill, x_fin_cg, r_fill),
    ]

    # WGHT의 MTOW == sum(breakdown) 항등식을 지키기 위해 여기서 직접 합산한다.
    m_str = sum(item.m for item in breakdown)

    # 초기 구현에서는 서포트/브림을 별도 계산하지 않는다.
    m_print = m_str

    # [이식 3/3] ICD0-007 의 StrcOut(m_str·g10) 대신 dict 로 돌려주고,
    # ICD0-008 매핑은 아래 run() 이 한 곳에서만 한다.
    return {"m_str": m_str, "m_print": m_print, "m_fill": m_fill,
            "w_fill": w_fill, "breakdown": breakdown, "g10": g10}


# ══════════════════════════════════════════════════════════════════════════
# ICD0-008 어댑터 — 여기서만 이름을 바꾼다
# ══════════════════════════════════════════════════════════════════════════
class _GeomShim:
    """ICD0-008 의 HullOut·AeroOut 을 담당자 코드가 기대하는 GeomOut 모양으로 비춘다.

    담당자 코드는  로 필드를 읽으므로, 이름만 맞춰 주면 본문을
    한 줄도 안 고치고 돌아간다. 이름 대응이 틀리면 **여기 한 곳만** 보면 된다.

        담당자(ICD0-007)          ICD0-008
        ---------------------    ------------------------------------------
        c_r, c_t                 hull.c_root, hull.c_tip
        t_wall                   geom.wall_thickness(dv)
        d_pod, l_pod, x_pod      geom.pod(...)  — 런처가 넘긴다 (모터 질량 의존)
        arm_rotor                hull.r_body + f_mount·hull.b_fin
        S_wet_body, l_body       그대로
        (없음)                   b_fin, x_wet_body, q_cr, CN_alpha_fin_1  ← 추가 공급
    """

    def __init__(self, dv: DesignVars, hl: HullOut, aer: AeroOut, pod=None):
        self.l_body = hl.l_body
        self.r_body = hl.r_body
        self.S_wet_body = hl.S_wet_body
        self.x_wet_body = hl.x_wet_body          # [이식 4/5] 쉘 도심
        self.c_r = hl.c_root
        self.c_t = hl.c_tip
        self.t_fin = hl.t_fin
        self.b_fin = hl.b_fin                    # [이식 1/5] 진짜 핀 스팬
        self.arm_rotor = hl.r_body + dv.f_mount * hl.b_fin
        self.t_wall = k.t_0 + k.k_t * (dv.n_design - k.n_ref_load)
        self.t_wall_pod = k.t_wall_pod

        # [이식 2/5] AERO 가 내는 값 — 담당자 코드의 자체 계산보다 우선한다.
        # 담당자 CN_alpha_fin 은 **핀 1장·S_fin_1 기준**이고 AERO 것은 4장 합·S_ref
        # 기준이라 정규화가 다르다. 같은 물리량(핀 1장의 법선력)이 되도록 환산한다.
        self.q_cr = aer.q_cr
        S_fin_1 = dv.S_fin / k.N_rot
        self.CN_alpha_fin_1 = (aer.CN_alpha_fin * aer.S_ref
                               / (k.N_rot * S_fin_1)) if S_fin_1 > 0 else 0.0

        # 포드는 모터 질량의 함수라 런처가 넘긴다. 없으면 포드 쉘 질량이 0 이 된다.
        if pod is None:
            self.d_pod = self.l_pod = 1.0e-6   # 담당 코드의 d_pod>0 검사 통과용.
        else:                                  # 질량은 run() 에서 0 으로 덮는다.
            self.d_pod, self.l_pod = float(pod[0]), float(pod[1])
        self.x_pod = dv.x_fin + k.f_pod_c * hl.c_root


# 담당자 코드의 타입 힌트에 ICD0-007 의 GeomOut 이 남아 있다. `from __future__ import
# annotations` 덕에 평가되진 않지만, 이름이 없으면 읽는 사람이 헷갈린다.
GeomOut = _GeomShim


def run(dv: DesignVars, hl: HullOut, aer: AeroOut, MTOW: float,
        pod: tuple | None = None) -> StrcOut:
    """① 프린트 구조 무게 — ICD0-008 진입점.

    pod : (d_pod, l_pod, …) [m]. `geom.pod` 의 반환을 그대로 넘기면 된다.
          모터 질량의 함수라 런처가 만들어 준다 (STRC 가 GEOM 을 부르면 §5 위반).
          None 이면 **포드 쉘 질량이 0 이 된다** — 값이 아니라 '아직 안 넘겼다'는 뜻이다.

    이름 대응:  담당자 m_str → W_str,   담당자 g10 → g5  (ICD0-007 번호였다)
    """
    geo = _GeomShim(dv, hl, aer, pod)
    fixed = fixed_masses(dv, geo)
    if pod is None:
        fixed["pod_shell"] = 0.0          # 치수를 모르면 지어내지 않는다
    out = _solve(dv, geo, MTOW, fixed)

    return StrcOut(W_str=out["m_str"], m_print=out["m_print"], g5=out["g10"],
                   breakdown_str=out["breakdown"],
                   m_fill=out["m_fill"], w_fill=out["w_fill"])
