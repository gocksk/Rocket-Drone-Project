"""STRC — 프린트 구조 질량과 핀 루트 1차 구조 사이징.

이 파일은 통합 사이징 코드의 초기 구현용 저차 모델이다.

계산 범위
---------
1. 동체 쉘, 핀 4장, 모터 포드 4개의 기본 구조 질량
2. 로터 추력과 고속 off-design 핀 공력에 의한 핀 루트 굽힘/전단
3. 필요한 핀 루트 100% 인필 보강 폭과 추가 질량
4. WGHT가 MTOW, x_cg, J_yy를 계산할 수 있도록 breakdown_str 반환

중요한 역할 분담
---------------
- STRC는 ``m_str``(ICD의 ``W_str``), ``m_print``, ``breakdown_str``을 반환한다.
- MTOW, x_cg, J_xx, J_yy, J_zz는 WGHT가 계산한다.
- STRC 내부에는 MTOW 수렴 반복문을 두지 않는다.

GEOM에 필요한 필드
-----------------
기존 필드:
    l_body, r_body, x_pod, arm_rotor
추가 필드:
    t_wall, t_fin, c_r, c_t, S_wet_body, d_pod, l_pod
선택 필드:
    t_wall_pod

AERO 입력은 사용하지 않는다.
- q_cr는 ``0.5 * rho_air * V_cr**2``로 STRC 내부 계산
- CN_alpha_fin은 핀 한 장의 aspect ratio로부터 저차 유한날개식으로 계산

현재 모델에서 제외하는 항목
--------------------------
링프레임, 격벽, 접합부 상세, 동체 좌굴, 로터 반작용 토크,
착륙 충격, 피로/진동, 상세 비틀림, FEM, 천이 시간 이력
"""

from __future__ import annotations

from math import ceil, hypot, pi, sqrt
from typing import Dict, List

import constants as k
from interfaces import DesignVars, GeomOut, MassItem, StrcOut

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
    arm_rotor = float(getattr(geo, "arm_rotor", float(dv.arm_rotor)))

    # 로터축이 핀 끝에 있는 초기 형상.
    b_fin = arm_rotor - r_body
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
        "n_rot": float(n_rot),
        "r_body": r_body,
        "arm_rotor": arm_rotor,
        "b_fin": b_fin,
        "S_fin_1": S_fin_1,
        "c_r": c_r,
        "c_t": c_t,
        "t_fin": t_fin,
        "root_arm": b_fin,
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

    q_cr = _cruise_dynamic_pressure()
    CN_alpha_fin = _fin_normal_slope(fin)
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


def run(
    dv: DesignVars,
    geo: GeomOut,
    MTOW: float,
    fixed: Dict[str, float] | None = None,
) -> StrcOut:
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
    x_shell = 0.5 * l_body
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

    # 현재 WGHT와 기존 interfaces.py가 사용하는 StrcOut 필드명에 맞춘다.
    # 물리적으로 m_str은 ICD의 W_str과 같은 값이다.
    return StrcOut(
        m_str=m_str,
        m_print=m_print,
        m_fill=m_fill,
        w_fill=w_fill,
        breakdown_str=breakdown,
        g10=g10,
    )
