"""AERO — 항력·정적안정. [골격 구현 + 스텁]
가이드라인: 「AERO 계산 가이드라인 — 항력·정적안정」
빌드업·Barrowman 구조는 가이드라인대로. [스텁] 표시(cd_2d)만 교체하면 된다.
"""
import math
import constants as k
from interfaces import DesignVars, GeomOut, AeroOut
from common.atm import atm


def cd_2d(Re, tc):
    """[스텁] 핀 익형 2D 항력계수 — NeuralFoil 테이블 보간으로 교체 (AERO §11 2층)."""
    return 0.009 * (1.0 + 2.0 * tc)


def _Cf(Re):
    return 0.455 / (math.log10(max(Re, 1e4))) ** 2.58


def run(dv: DesignVars, geo: GeomOut) -> AeroOut:
    air = atm(0.0, 0.0)
    q_cr = 0.5 * air.rho * k.V_cr ** 2

    # §3 성분별 항력 빌드업 (S_ref 기준 환산)
    def CD0(V):
        Re_b = air.rho * V * geo.l_body / air.mu
        Re_f = air.rho * V * geo.c_r / air.mu
        Re_p = air.rho * V * geo.l_pod / air.mu
        FF_body = 1.0 + 60.0 / dv.lambda_body ** 3 + 0.0025 * dv.lambda_body
        FF_pod = 1.0 + 0.35 / k.f_pod
        cd_body = _Cf(Re_b) * FF_body * (geo.S_wet["nose"] + geo.S_wet["cyl"]) / geo.S_ref
        cd_base = k.k_base * geo.S_base / geo.S_ref
        cd_fin = cd_2d(Re_f, k.tc_fin) * geo.S_wet["fin"] / geo.S_ref
        cd_pod = _Cf(Re_p) * FF_pod * geo.S_wet["pod"] / geo.S_ref
        cd_int = geo.N_junc * k.k_int * geo.t_fin ** 2 / geo.S_ref
        return cd_body + cd_base + cd_fin + cd_pod + cd_int

    def F_drag(V):
        return k.k_cal * 0.5 * air.rho * V ** 2 * geo.S_ref * CD0(max(V, 1.0))

    # §5 Barrowman
    CN_nose = 2.0
    x_nose = k.k_cp_nose * geo.l_nose
    K_fb = 1.0 + geo.r_body / (geo.b_1 + geo.r_body)
    l_mid = math.sqrt(geo.b_1 ** 2 + (geo.x_t + (geo.c_t - geo.c_r) / 2.0) ** 2)
    CN_fin = (K_fb * 4.0 * k.N_rot * (geo.b_1 / dv.d_body) ** 2
              / (1.0 + math.sqrt(1.0 + (2.0 * l_mid / (geo.c_r + geo.c_t)) ** 2)))
    x_fin_cp = (geo.x_fin
                + geo.x_t * (geo.c_r + 2.0 * geo.c_t) / (3.0 * (geo.c_r + geo.c_t))
                + (1.0 / 6.0) * ((geo.c_r + geo.c_t)
                                 - geo.c_r * geo.c_t / (geo.c_r + geo.c_t)))
    CN_alpha = CN_nose + CN_fin
    x_cp = (CN_nose * x_nose + CN_fin * x_fin_cp) / CN_alpha

    # §6 고받음각 확장 — 평면형 도심과 가중평균
    A_nose = k.k_side * dv.d_body * geo.l_nose
    A_cyl = dv.d_body * geo.l_cyl
    A_fin = k.k_finproj * dv.S_fin
    A_plan = A_nose + A_cyl + A_fin
    x_cp_cross = (A_nose * (k.k_xn * geo.l_nose)
                  + A_cyl * (geo.l_nose + geo.l_cyl / 2.0)
                  + A_fin * (geo.x_fin + geo.c_r / 2.0)) / A_plan

    def C_N(alpha):
        pot = CN_alpha * math.sin(alpha) * math.cos(alpha)
        cross = k.eta_cf * k.Cd_c * (A_plan / geo.S_ref) * math.sin(alpha) ** 2
        return pot + cross

    def x_cp_alpha(alpha):
        pot = CN_alpha * math.sin(alpha) * math.cos(alpha)
        cross = k.eta_cf * k.Cd_c * (A_plan / geo.S_ref) * math.sin(alpha) ** 2
        tot = pot + cross
        return (pot * x_cp + cross * x_cp_cross) / tot if tot > 1e-9 else x_cp

    return AeroOut(F_drag=F_drag, CN_alpha=CN_alpha, x_cp=x_cp,
                   CN_alpha_fin=CN_fin, q_cr=q_cr,
                   C_N=C_N, x_cp_alpha=x_cp_alpha, x_cp_cross=x_cp_cross,
                   CD0_cr=CD0(k.V_cr))
