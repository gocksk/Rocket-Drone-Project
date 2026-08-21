"""PROP-A — 추진 성능표 제작. [스텁]
가이드라인: 「PROP-A 계산 가이드라인 — 추진 성능표 제작」
[스텁] 추진 물리는 전부 임의 근사다. PROP 담당이 BEMT 계수 테이블
(C_T·C_Q(J, pd)) + 이분법 평형 풀이로 교체한다 (가이드라인 §2~§6).
인터페이스(query/T_avail/U_pack 서명)는 유지할 것.
"""
import math
import constants as k
from interfaces import DesignVars, PropAOut
from common import srl
from common.atm import atm


def run(dv: DesignVars, parts, F_drag) -> PropAOut:
    air = atm()
    mot, pr, bt = parts["motor"], parts["prop"], parts["batt"]

    # 배터리 전압 모델 (구조는 실제, 값은 SRL 스텁 의존)
    def U_pack(SOC, I_pack):
        return dv.n_ser * srl.U_ocv(SOC) - I_pack * bt.R_pack

    I_guess = 30.0
    U_eval = U_pack(1.0 - k.DoD, I_guess)          # 방전 말기 최악 전압 (ICD §7)

    # [스텁] 프롭 추력·토크 — 단순 감쇠 모델 (BEMT 테이블로 교체)
    CT0, CQ0, J_lapse = 0.15, 0.020, 1.20

    def _n_max(U):
        return dv.kv_mot * U / 60.0                # [rev/s]

    def _T1(V, n):
        J = V / max(n * dv.d_prop, 1e-6)
        f = max(0.0, 1.0 - J / (J_lapse * dv.pd_prop))
        return k.k_T_cal * CT0 * f * air.rho * n ** 2 * dv.d_prop ** 4

    def _Q1(V, n):
        J = V / max(n * dv.d_prop, 1e-6)
        f = max(0.05, 1.0 - 0.7 * J / (J_lapse * dv.pd_prop))
        return k.k_Q_cal * CQ0 * f * air.rho * n ** 2 * dv.d_prop ** 5

    def T_avail(V, U):
        n = 0.92 * _n_max(U)                       # [스텁] 부하 시 회전수 근사
        return k.N_rot * _T1(V, n)

    def query(V, T_req_1, U):
        """(V, 로터당 요구추력, 전압) → (P_elec, I_mot, Q_prop) — [스텁] 역산."""
        n_hi = _n_max(U)
        n = n_hi * math.sqrt(max(T_req_1, 0.01) / max(_T1(V, n_hi), 1e-6))
        n = min(n, n_hi)
        Q = _Q1(V, n)
        Kt = 60.0 / (2.0 * math.pi * dv.kv_mot)    # 토크상수 (SI 변환 필수!)
        I_mot = Q / Kt + mot.I_0
        P_elec = (I_mot * mot.R_mot + 2.0 * math.pi * n / (dv.kv_mot * 2 * math.pi / 60.0)) \
                 * I_mot / k.eta_esc
        return P_elec, I_mot, Q

    # 전개 최대 작동점 기준 (g4·g5는 무게 무관 — PROP-A §8)
    n_full = 0.92 * _n_max(U_eval)
    _, I_top, _ = query(0.0, _T1(0.0, n_full), U_eval)
    I_max = I_top
    I_pack_max = k.N_rot * I_max / k.eta_esc
    Q_pack = dv.E_batt / (dv.n_ser * k.U_cell_nom)
    g4 = k.c_rate_max * Q_pack / max(I_pack_max, 1e-6) - 1.0
    M_tip = math.sqrt((math.pi * dv.d_prop * n_full) ** 2 + (1.3 * k.V_cr) ** 2) / air.a_snd
    g5 = k.M_tip_max / M_tip - 1.0

    m_esc = srl.esc(I_max * k.k_margin).m
    m_propsys = k.N_rot * (mot.m + pr.m + m_esc) * (1.0 + k.k_wire)

    return PropAOut(query=query, T_avail=T_avail, U_pack=U_pack, U_eval=U_eval,
                    m_propsys=m_propsys, I_max=I_max, g4=g4, g5=g5)
