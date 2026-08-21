"""통합 실행 — DSM-002의 3구획을 그대로 코드로 옮긴 것.

① 한 번만 (무게 무관)  : SRL → GEOM → AERO → PROP-A   [즉시 탈락: g9·클리어런스·g4·g5]
② 반복 (무게 맞추기)   : STRC ↔ WGHT                   [피드백 = MTOW 하나]
③ 마지막에 한 번        : PROP-B → MISS → STAB → COST  [성적표·합격조건]

[규칙] 이 파일과 interfaces.py는 통합 담당만 수정한다.
"""
from interfaces import DesignVars, Result
from common import srl
from modules import geom, aero, prop_a, wght, prop_b, miss, stab, cost
import constants as k


def evaluate(dv: DesignVars) -> Result:
    r = Result(feasible=True)

    # ══ ① 전처리 — 무게 무관, 설계점당 1회 ══
    parts = {"motor": srl.motor(dv.kv_mot, dv.d_stat, dv.h_stat),
             "prop": srl.prop(dv.d_prop, dv.pd_prop),
             "esc": srl.esc(40.0),                    # 치수용 잠정 — PROP-A가 정격 확정
             "batt": srl.batt(dv.E_batt, dv.n_ser)}

    r.geom = geom.run(dv, parts)
    r.g["g9"], r.g["g_clear"] = r.geom.g9, r.geom.g_clear
    if r.geom.g9 < 0 or r.geom.g_clear < 0:
        r.feasible, r.fail_stage = False, "① GEOM (내장/클리어런스)"
        return r

    r.aero = aero.run(dv, r.geom)
    r.propa = prop_a.run(dv, parts, r.aero.F_drag)
    r.g["g4"], r.g["g5"] = r.propa.g4, r.propa.g5
    if r.propa.g4 < 0 or r.propa.g5 < 0:
        r.feasible, r.fail_stage = False, "① PROP-A (방전한계/팁마하)"
        return r

    # ══ ② 수렴 루프 — {STRC ↔ WGHT} ══
    r.wght = wght.converge(dv, r.geom, r.aero, r.propa.m_propsys)
    if not r.wght.converged:
        r.feasible, r.fail_stage = False, "② WGHT (수렴 실패 — 해석 실패로 분류)"
        return r
    r.g["g10"] = r.wght.strc.g10

    # ══ ③ 후처리 — 수렴 후 1회 ══
    r.propb = prop_b.run(dv, r.geom, r.aero, r.propa, r.wght.MTOW)
    r.g["g1"], r.g["g2"] = r.propb.g1, r.propb.g2

    r.miss = miss.run(dv, r.geom, r.aero, r.propa, r.propb, r.wght.MTOW)
    r.g["g8"] = r.miss.g8

    r.stab = stab.run(dv, r.geom, r.aero, r.wght, r.propb)
    r.g["g6"], r.g["g7"] = r.stab.g6, r.stab.g7

    r.cost = cost.run(dv, parts, r.propa.I_max, r.wght.strc.m_print, r.miss.E_req)

    # ══ 성적표 (EC C1~C8) ══
    r.ec = {"C1_MTOW[kg]": r.wght.MTOW,
            "C2_R_dash[m]": r.miss.R_dash,
            "C3_margin_V": r.propb.margin_V,
            "C4_n_design": dv.n_design,
            "C5_alpha_max[rad/s2]": r.stab.alpha_max,
            "C6_SPL_hover[dB]": r.propb.SPL_hover,
            "C7_Cost_acq[KRW]": r.cost.Cost_acq,
            "C8_l_body[m]": r.geom.l_body}
    r.feasible = all(v >= 0 for v in r.g.values())
    return r


def report(r: Result):
    print("=" * 58)
    if r.fail_stage:
        print(f"조기 탈락: {r.fail_stage}")
    print("합격 조건 (음수 = 불합격)")
    for name in ["g1", "g2", "g4", "g5", "g6", "g7", "g8", "g9", "g10", "g_clear"]:
        if name in r.g:
            mark = "OK " if r.g[name] >= 0 else "FAIL"
            print(f"  {name:8s} {r.g[name]:+8.3f}  {mark}")
    if r.ec:
        print("-" * 58)
        print("성적표 (EC)")
        for name, v in r.ec.items():
            print(f"  {name:22s} {v:12.3f}")
    if r.wght:
        print("-" * 58)
        print(f"수렴: {r.wght.n_iter}회 반복 · x_cg={r.wght.x_cg:.3f} m"
              f" · J_yy={r.wght.J_yy * 1e3:.2f} g·m²")
    if r.propb:
        import math
        print(f"진단: 순항 자세각 θ_req = {math.degrees(r.propb.theta_req_cr):.1f}°"
              f" (공력 모델 가정 밖이면 트림 업그레이드 검토 — PROP-B §2)")
    print("=" * 58)


if __name__ == "__main__":
    # 예시 설계점 — 숫자는 스모크 테스트용 (의미 없음)
    dv = DesignVars(
        d_body=0.09, lambda_body=7.0, S_fin=0.036, x_fin=0.50,
        AR_fin=2.2, f_mount=0.8, n_design=4.0,
        kv_mot=2000.0, d_stat=0.028, h_stat=0.008,
        d_prop=0.13, pd_prop=1.30, E_batt=80.0, n_ser=6,
    )
    report(evaluate(dv))
