"""PROP-B — 성능 읽기. [구조 실제 구현 — 값은 PROP-A 스텁에 의존]
가이드라인: 「PROP-B 계산 가이드라인 — 성능 읽기」
성적 1위 margin_V가 여기서 나온다. 요구추력 √(D²+W²) 규약 (ICD §7).
"""
import math
import constants as k
from interfaces import DesignVars, PropBOut
from common.atm import atm


def run(dv: DesignVars, geo, aero, pa, MTOW) -> PropBOut:
    air = atm()
    W = MTOW * k.g
    U = pa.U_eval
    k_block = 0.95   # ★ 핀 블로케이지 — PROP+GEOM 협의로 확정 (잠정)

    def T_req(V):
        return math.sqrt(aero.F_drag(V) ** 2 + W ** 2)

    # §2 순항 자세각 (진단 — 규약 건전성 지표)
    theta_req_cr = math.atan2(W, aero.F_drag(k.V_cr))

    # §4 호버 (g2 먼저 — 뜨지도 못하면 최고속도가 무의미)
    T_static = pa.T_avail(0.0, U) * k_block
    tw_hover = T_static / W
    g2 = tw_hover / k.tw_min - 1.0
    T_h1 = W / (k.N_rot * k_block)
    P1, I1, Q_op = pa.query(0.0, T_h1, U)
    P_hover = k.N_rot * P1

    # §3 최고속도 교점 — 이분법 (Δ(V)=T_avail−T_req 단조)
    if pa.T_avail(0.0, U) <= T_req(0.0):
        V_max = 0.0
    else:
        lo, hi = 0.0, 1.3 * k.V_cr
        if pa.T_avail(hi, U) > T_req(hi):
            V_max = hi                        # 탐색 상한에서도 여유 → 상한으로 클램프
        else:
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if pa.T_avail(mid, U) > T_req(mid):
                    lo = mid
                else:
                    hi = mid
            V_max = 0.5 * (lo + hi)
    margin_V = V_max / k.V_cr
    g1 = margin_V / (1.0 + k.eps_snap) - 1.0

    # §5 차동추력·차동토크 여유
    T_op = W / k.N_rot
    dT_up = pa.T_avail(0.0, U) * k_block / k.N_rot - T_op
    dT_down = T_op - 0.0                     # T_min = 0 (고정피치·단방향)
    dT_max = max(min(dT_up, dT_down), 0.0)
    _, _, Q_hi = pa.query(0.0, (T_op + dT_max) / k_block, U)
    dQ_max = abs(Q_hi - Q_op)

    # §6 호버 소음 — 앵커 스케일링
    n_hover = dv.kv_mot * U / 60.0 * math.sqrt(max(T_h1, 0.01) /
              max(pa.T_avail(0.0, U) / k.N_rot, 1e-6))          # [스텁] 근사
    V_tip = math.pi * dv.d_prop * n_hover
    SPL = (k.SPL_ref
           + 60.0 * math.log10(max(V_tip, 1.0) / k.V_tip_ref)   # k_tip=60 ★
           + 10.0 * math.log10(max(T_h1, 0.1) / k.T_ref)
           - 20.0 * math.log10(k.r_obs / k.r_ref)
           + 10.0 * math.log10(k.N_rot / k.N_ref))

    return PropBOut(V_max=V_max, margin_V=margin_V, P_hover=P_hover,
                    n_hover=n_hover, dT_max=dT_max, dQ_max=dQ_max,
                    SPL_hover=SPL, theta_req_cr=theta_req_cr, g1=g1, g2=g2)
