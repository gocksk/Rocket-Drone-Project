"""GEOM — 형상·배치. [가이드라인 수식 구현 — 담당 검토·보완 필요]
가이드라인: 「GEOM 계산 가이드라인 — 형상·배치」 (개정 002)
순수 대수라 스텁 대신 가이드라인 §2~§7을 그대로 옮겼다.
GEOM 담당은 식이 문서와 일치하는지 검토하고, 빠진 세부를 보완한다.
"""
import math
import constants as k
from interfaces import DesignVars, GeomOut
from common import srl


def run(dv: DesignVars, parts) -> GeomOut:
    mot, es, bt = parts["motor"], parts["esc"], parts["batt"]

    # §2 동체 기본 치수
    l_body = dv.lambda_body * dv.d_body
    l_nose = k.f_nose * dv.d_body
    l_cyl = l_body - l_nose
    r_body = dv.d_body / 2.0
    S_ref = math.pi * dv.d_body ** 2 / 4.0
    S_base = S_ref

    # §3 핀 제원 전개
    S_1 = dv.S_fin / k.N_rot
    b_1 = math.sqrt(dv.AR_fin * S_1)
    c_r = 2.0 * S_1 / (b_1 * (1.0 + k.lam_fin))
    c_t = k.lam_fin * c_r
    x_t = c_r - c_t
    t_fin = k.tc_fin * c_r

    # §4 포드·프롭 배치 (핀 결합)
    arm_rotor = r_body + dv.f_mount * b_1
    root_arm = dv.f_mount * b_1
    d_pod = dv.d_stat + 2.0 * k.t_pod
    l_pod = max(k.f_pod * d_pod, dv.h_stat + es.l_esc + 0.005)
    x_pod = dv.x_fin + k.f_pod_c * c_r
    x_prop = x_pod + l_pod / 2.0 + k.d_hub

    # §5 면적 집계 (암 항 없음 — 핀 일체형)
    S_wet = {
        "nose": k.k_nose * math.pi * dv.d_body * l_nose,
        "cyl": math.pi * dv.d_body * l_cyl,
        "fin": 2.0 * dv.S_fin * k.k_thk,
        "pod": k.N_rot * math.pi * d_pod * l_pod * k.k_form,
    }
    A_front_pod = k.N_rot * math.pi * d_pod ** 2 / 4.0
    N_junc = 2 * k.N_rot

    # §6 내부 공간과 부품 배치 (순서 고정: 카메라·센서 → 배터리 → FC/ESC)
    t_wall = k.t_0 + k.k_t * (dv.n_design - k.n_ref_load)
    d_int = dv.d_body - 2.0 * t_wall
    l_int = l_cyl - 2.0 * k.d_end
    avio = srl.avio()
    cam_L = sum(a[2] for a in avio if a[0] in ("camera", "sensor"))
    fc_L = sum(a[2] for a in avio if a[0] in ("fc", "rx_vtx"))
    seq = [("cam_sensor", cam_L, max(a[3] for a in avio), max(a[4] for a in avio)),
           ("batt", bt.L, bt.W, bt.H),
           ("fc_esc", fc_L, 0.030, 0.014)]
    x_parts, x_cur = {}, l_nose + k.d_end
    for name, L, _, _ in seq:
        x_parts[name] = x_cur + L / 2.0
        x_cur += L

    # §7 합격 조건
    diag_max = max(math.sqrt(w ** 2 + h ** 2) for _, _, w, h in seq)
    sum_L = sum(L for _, L, _, _ in seq)
    g9 = min(0.95 * d_int / diag_max - 1.0,
             0.95 * l_int / max(sum_L, 1e-9) - 1.0)
    g_adj = math.sqrt(2.0) * arm_rotor / (dv.d_prop * (1.0 + k.d_clr)) - 1.0
    g_body = ((arm_rotor - dv.d_prop / 2.0) / (r_body * (1.0 + k.d_clr)) - 1.0
              if x_prop < l_body else 1.0)
    g_ax = (x_prop - (dv.x_fin + c_r + k.d_ax)) / l_body
    g_clear = min(g_adj, g_body, g_ax)

    return GeomOut(l_body=l_body, l_nose=l_nose, l_cyl=l_cyl, r_body=r_body,
                   S_ref=S_ref, S_base=S_base, S_wet=S_wet,
                   A_front_pod=A_front_pod, N_junc=N_junc,
                   b_1=b_1, c_r=c_r, c_t=c_t, x_t=x_t, t_fin=t_fin,
                   x_fin=dv.x_fin, d_pod=d_pod, l_pod=l_pod,
                   x_pod=x_pod, x_prop=x_prop,
                   arm_rotor=arm_rotor, root_arm=root_arm,
                   t_wall=t_wall, d_int=d_int, x_parts=x_parts,
                   g9=g9, g_clear=g_clear)
