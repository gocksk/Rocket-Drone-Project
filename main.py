"""통합 런처 — ICD0-008 §1 의 ⓪①② 를 그대로 코드로 옮긴 것.

  ⓪ 전처리    — 무게·부품과 무관, 설계점당 1회
       atm.run → geom.hull → aero.run → prop.build_map
  ① 사이징 루프 — MTOW 하나를 수렴시킨다
       resp_of(MTOW): prop.size_motor → miss.required_energy → strc.run
       wght.converge(m_fixed, resp_of)
  ② 후처리    — 수렴값으로 1회
       geom.layout → geom.check_fit → prop.evaluate → miss.achieved_range
       → wght.mass_props → stab.run → cost.run

[규칙] 모듈은 서로를 부르지 않는다. 여기가 조립한다.
       유일한 예외는 PROP → THRM (§5).
       이 파일과 interfaces.py 는 통합 담당만 수정한다.
"""
import constants as k
from common.out import stdout_utf8
from interfaces import (DesignVars, PreOut, RespPayload, Result, MassItem,
                        FAIL_NONE, FAIL_GEOM, FAIL_G1, FAIL_G2, FAIL_G3,
                        FAIL_G4_STRUCTURAL, FAIL_G4_NUMERICAL, FAIL_G4_MAXITER)
from modules.geom import GeomInfeasible
from modules import atm, geom, aero, prop, thrm, miss, strc, wght, stab, cost


# ══════════════════════════════════════════════════════════════════════════
# ⓪ 전처리 — 단일 출처 (ICD §8 C-1). 검증 도구도 이 함수를 부른다.
# ══════════════════════════════════════════════════════════════════════════
def preprocess(dv: DesignVars) -> PreOut:
    """무게·부품과 무관한 계산. 설계점당 1회.

    이 넷은 MTOW 를 되먹임 입력으로 참조하지 않는다 — 참조하게 만드는 변경은
    ICD §9 에서 원칙 반려다.
    """
    air = atm.run(k.h_miss)
    hl = geom.hull(dv)
    aer = aero.run(dv, hl, air)
    pmap = prop.build_map(dv, air)
    return PreOut(atm=air, hull=hl, aero=aer, pmap=pmap)


# ══════════════════════════════════════════════════════════════════════════
# 탈락 처리 — 사유 코드를 결과까지 그대로 전달한다 (§5 ▶수렴·발산 분류)
# ══════════════════════════════════════════════════════════════════════════
def _fail(r: Result, code: str, stage: str) -> Result:
    """'설정 탓'과 '물리 탓'을 한 문자열로 뭉개지 않는다 — DOE 로그의 1급 데이터."""
    r.feasible = False
    r.fail_code = code
    r.fail_stage = stage
    return r


def _close_U_eval(outputs, U_hi: float):
    """§4.5 U_eval 순환을 닫는다 — 구간 유지 regula falsi (Illinois 변형).

    잔차  f(U) = U_eval(E_batt(U), I_dash(U)) − U  는 U 에 **단조 감소**다:
    전압을 낮게 잡으면 사이징이 kv 를 높여 전류가 늘고, 그만큼 팩 강하가 커진다.

    이분법과 같은 안전성(항상 구간을 유지)에 수렴 속도만 올린 것이다 —
    잔차가 매끄러워 이분법 11회가 4~5회로 준다. 결정론적이므로 resp_of 의
    순수성은 그대로다 (같은 MTOW → 같은 U → 같은 응답질량).

    브래킷 하한은 고정 격자를 훑어 찾는다. 끝까지 부호가 안 바뀌면 그 설계점은
    방전 말기에 dash 를 버틸 수 없다는 뜻이라 해가 없다.
    반환: (U, outputs(U) 결과, 반복 횟수).  해가 없으면 반복 횟수 -1.
    """
    f_hi, hi_out = outputs(U_hi)
    f_hi -= U_hi
    if f_hi >= 0.0:
        return U_hi, hi_out, 0            # 강하가 없어도 성립 — 그대로 쓴다

    lo = None
    for i in range(1, k.n_U_scan + 1):    # 고정 격자 — 결정론적
        U = U_hi * (1.0 - i / (k.n_U_scan + 1.0))
        f, out = outputs(U)
        f -= U
        if f > 0.0:
            lo, f_lo, lo_out = U, f, out
            break
    if lo is None:
        return U_hi, hi_out, -1           # 해 없음 — 말기 전압으로 dash 불가

    side = 0
    for it in range(1, k.N_U_max + 1):
        U = (lo * f_hi - U_hi * f_lo) / (f_hi - f_lo)     # regula falsi
        f, out = outputs(U)
        f -= U
        if abs(f) < k.eps_U or abs(U_hi - lo) < k.eps_U:
            return U, out, it
        if f > 0.0:
            lo, f_lo, lo_out = U, f, out
            if side == +1:
                f_hi *= 0.5               # Illinois — 한쪽에 갇히는 것을 막는다
            side = +1
        else:
            U_hi, f_hi, hi_out = U, f, out
            if side == -1:
                f_lo *= 0.5
            side = -1
    return U, out, k.N_U_max


# WGHT status → 탈락 사유 코드. limit_cycle 은 탈락이 아니다 (이산화 한계까지 수렴).
_WGHT_FAIL = {
    "diverged_structural": FAIL_G4_STRUCTURAL,   # 무게 스노우볼 — 성립 불가 설계점
    "diverged_numerical": FAIL_G4_NUMERICAL,     # beta 선택 문제 — 재시도 가능
    "max_iter": FAIL_G4_MAXITER,                 # 반복 소진 — 잔여 오차 동반 보고
}


def evaluate(dv: DesignVars) -> Result:
    r = Result()

    # ══ ⓪ 전처리 ══
    try:
        r.pre = pre = preprocess(dv)
    except GeomInfeasible as e:
        # 형상이 성립하지 않는 설계점. g 를 새로 만들지 않고 사유 코드로만 전달한다.
        # DOE 배치가 예외로 죽는 대신 한 줄의 탈락 기록으로 남는다.
        r.diag["geom_reason"] = str(e)
        return _fail(r, FAIL_GEOM, "⓪ GEOM.hull — 형상 불성립")

    r.g["g1"] = pre.pmap.g1
    r.diag["V_pitch"] = pre.pmap.V_pitch
    # §2 하한 규칙 여유 — g1(판정)과 별개로 "표본이 규칙 안에 있었나"를 로그에 남긴다
    r.diag["pd_min"] = prop.pd_prop_min(k.V_cr, pre.atm.a_snd)
    r.diag["pd_margin"] = dv.pd_prop / r.diag["pd_min"]
    if r.g["g1"] < 0:
        # g1–g4 는 사이징 성립 조건이다. 위반하면 뒤를 계산하지 않는다 (§6.1).
        return _fail(r, FAIL_G1, "⓪ PROP.build_map — 순항 미성립")

    # ══ ① 사이징 루프 ══
    # 고정 질량 = MTOW 와 무관한 질량.
    # ICD §5 의사코드는 W_pl + W_avio 만 적었으나 프롭 질량도 MTOW 무관이라
    # 여기 들어가야 한다 (build_map 이 ⓪ 출력으로 낸다) — [확정 필요]
    m_avio = sum(a[1] for a in k.AVIO_LIST)
    m_fixed = k.W_pl + m_avio + k.N_rot * pre.pmap.m_prop

    # 버스 전압 — §4.5 는 U_eval = U_cell(1−DoD)·n_ser − I_dash·R_pack 인데
    # I_dash 는 size_motor 출력이고 R_pack 은 E_batt 의 함수라 순환이다.
    # [스텁] P4·P5 에서 갱신 방식을 정하기 전까지 I·R 항을 뺀 값으로 둔다. [결정 필요]
    U_bus = prop.U_ocv(1.0 - k.DoD) * dv.n_ser

    # 포드 치수를 모터 질량에서 만드는 클로저 — PROP 이 GEOM 을 직접 부르면 새 모듈 간
    # 호출이 되므로(§5) 런처가 조립해 넘긴다. resp_of 와 같은 패턴이다.
    def pod_of(m_mot: float):
        D_m, L_m, _ = thrm.motor_geometry(m_mot)
        d_pod, l_pod, _, _ = geom.pod(m_mot, D_m, L_m, pre.hull, dv)
        return d_pod, l_pod

    def resp_of(MTOW: float):
        """MTOW → (응답질량 합, payload). **순수 함수** — 캐시·전역·난수 금지.

        모듈 내부 순환(size_motor, required_energy)은 결정론적 이분법으로 닫는다.
        직전 반복값을 기억하면 배치 실행에서 설계점끼리 오염된다 (§5 ▶모듈 내부 순환).
        """
        # §4.5 U_eval 순환을 **여기서** 닫는다 (P5.6 확정). 모터는 U_eval 로 사이징
        # 돼야 하는데 U_eval 은 그 모터가 내는 I_dash 와 E_batt 에 달려 있다.
        # 열어 두면 사이징 전압(21.1 V)과 평가 전압(19.0 V)이 갈라져 C3 가 1 밑으로
        # 떨어지고 k_mot·형상이 margin_V 에 반응하지 않는다.
        def outputs(U):
            sm_ = prop.size_motor(MTOW, pre.pmap, pre.aero, pre.atm, U, dv.k_mot,
                                  pod_of=pod_of)
            re_ = miss.required_energy(MTOW, sm_.m_mot, sm_.kv, dv, pre.pmap,
                                       pre.aero, pre.atm, pod=pod_of(sm_.m_mot))
            return prop.U_eval(re_.E_batt, dv.n_ser, sm_.I_dash), (sm_, re_)

        U_ev, (sm, re), n_U = _close_U_eval(outputs, U_bus)
        st = strc.run(dv, pre.hull, pre.aero, MTOW, pod=pod_of(sm.m_mot))
        # m_mot 은 1기 기준으로 본다 — 사이징은 모터 한 개를 푸는 것이므로.
        # ICD §5 의사코드는 m_mot 을 그대로 더하고 있어 기수 곱이 빠져 있다 — [확정 필요]
        m_resp = k.N_rot * sm.m_mot + re.m_batt + re.m_pack + st.W_str
        return m_resp, RespPayload(m_mot=sm.m_mot, m_batt=re.m_batt,
                                   m_pack=re.m_pack, E_batt=re.E_batt,
                                   W_str=st.W_str, smot=sm, reqE=re, strc=st,
                                   U_eval=U_ev, n_U=n_U)

    r.wght = w = wght.converge(m_fixed, resp_of)
    pl: RespPayload = w.payload
    r.diag.update({"wght_status": w.status, "S_hat": w.S_hat, "err": w.err,
                   "n_iter": w.n_iter,
                   "active_motor": pl.smot.active, "active_batt": pl.reqE.active,
                   "n_bisect_mot": pl.smot.n_bisect, "n_bisect_E": pl.reqE.n_bisect,
                   "E_energy": pl.reqE.E_energy, "E_power": pl.reqE.E_power,
                   "I_max": pl.reqE.I_max})
    r.diag["S_split"] = wght.growth_split(w.history, None)

    # 모터 회귀는 조사 범위 밖에서 무효다 (§8 A-2). 사이징 결과가 범위를 벗어나면
    # R_mot·I0 가 외삽값이고 그 위에 g3(열 판정)이 얹혀 있으므로 로그에 남긴다.
    r.diag["kv"] = pl.smot.kv
    r.diag["T_peak"] = pl.smot.T_peak
    r.diag["thr_hover"] = pl.smot.thr_hover
    r.diag["P_shaft_dash"] = pl.smot.P_shaft_dash
    r.diag["fit_range"] = (
        "OK" if (k.m_mot_fit_lo <= pl.m_mot <= k.m_mot_fit_hi
                 and k.kv_fit_lo <= pl.smot.kv <= k.kv_fit_hi)
        else f"외삽 (m={pl.m_mot*1e3:.0f}g, kv={pl.smot.kv:.0f})")

    r.g["g2"], r.g["g3"] = pl.smot.g2, pl.smot.g3
    r.g["g4"] = w.g4
    if r.g["g2"] < 0:
        return _fail(r, FAIL_G2, "① PROP.size_motor — 팁 마하 초과")
    if r.g["g3"] < 0:
        return _fail(r, FAIL_G3, "① PROP.size_motor(←THRM) — 열 한계 초과")
    if r.g["g4"] < 0:
        return _fail(r, _WGHT_FAIL.get(w.status, FAIL_G4_MAXITER),
                     f"① WGHT.converge — {w.status}")

    # g5 부터는 설계 품질 조건이다. 계산은 끝까지 돌고 합격/불합격만 기록한다 (§6.1).
    r.g["g5"] = pl.strc.g5

    # ══ ② 후처리 ══
    # 부품 치수 — 밀도로 부피를 내고 동체 내부 원에 내접하는 상자로 만든다.
    # 모터 치수는 THRM 이 이미 열용량·표면적용으로 환산해 둔 것을 그대로 쓴다
    # (GEOM 이 THRM 을 부르면 새 모듈 간 직접 호출이 되므로 런처가 옮긴다, §5).
    d_int = geom.d_internal(dv, pre.hull)
    D_mot, L_mot, _ = thrm.motor_geometry(pl.m_mot)
    dims = {
        "batt": geom.box_from_volume((pl.m_batt + pl.m_pack) / k.rho_pack, d_int),
        "payload": geom.box_from_volume(k.W_pl / k.rho_payload, d_int),
        "motor": (L_mot, D_mot, D_mot),
        **{a[0]: (a[2], a[3], a[4]) for a in k.AVIO_LIST},
    }
    r.layout = lay = geom.layout(dv, pre.hull, dims)
    # 단면 검사는 데이터시트 치수를 가진 품목만 본다 — batt·payload 는 내부 원에
    # 내접하도록 만들어져 대각선이 정의상 d_int 라 검사가 무의미해진다.
    r.fit = fit = geom.check_fit(dv, pre.hull, lay, dims,
                                 fixed_section={a[0] for a in k.AVIO_LIST})
    r.g["g6"], r.g["g7"] = fit.g6, fit.g7
    r.diag.update({"d_int": d_int, "arm_rotor": lay.arm_rotor,
                   "L_batt": dims["batt"][0], "l_int": pre.hull.l_cyl - 2 * k.d_end})

    pod = pod_of(pl.m_mot)
    # 평가 전압은 ① 이 순환을 닫아 얻은 값 그대로다 — 사이징과 평가의 기준이 같아야
    # margin_V 가 의미를 갖는다 (§4.5).
    r.eval = ev = prop.evaluate(w.MTOW, pl.m_mot, pl.smot.kv, pl.E_batt, dv.n_ser,
                                pl.smot.I_dash, pre.pmap, pre.aero, pre.atm, pod=pod)
    r.rng = rng = miss.achieved_range(w.MTOW, pl.m_mot, pl.smot.kv, pl.E_batt, dv,
                                      pre.pmap, pre.aero, pre.atm, pod=pod)
    r.diag.update({"V_max": ev.V_max, "P_hover": ev.P_hover,
                   "d_pod": pod[0], "l_pod": pod[1],
                   "U_eval": pl.U_eval, "n_U": pl.n_U})

    # 질량 분해표 — 위치가 붙어야 무게중심·관성이 나오므로 배치(②) 뒤다.
    # Σ = MTOW 항등이 깨지면 wght.mass_props 가 즉시 멈춘다.
    # [확정 필요] 분해표 조립의 소유권 — ICD §5.1 은 WGHT 출력으로 적었으나
    #             품목 이름·위치를 아는 것은 런처와 GEOM 이다.
    x = lay.x_parts
    bd = list(pl.strc.breakdown_str)          # bd[0] = 동체 쉘 (mass_props 의 길이항)
    bd += [
        MassItem("motors", k.N_rot * pl.m_mot, x.get("motor", 0.0), lay.arm_rotor),
        MassItem("props", k.N_rot * pre.pmap.m_prop, x.get("motor", 0.0), lay.arm_rotor),
        MassItem("batt", pl.m_batt + pl.m_pack, x.get("batt", 0.0), 0.0),
        MassItem("payload", k.W_pl, x.get("payload", 0.0), 0.0),
    ]
    bd += [MassItem(a[0], a[1], x.get(a[0], 0.0), 0.0) for a in k.AVIO_LIST]
    r.mass = mp = wght.mass_props(w.MTOW, bd, pre.hull.l_body)

    # 조종 여유 — 천이 구간에서 로터 1기가 **더** 낼 수 있는 추력.
    # STAB 이 PROP 을 직접 부르면 새 모듈 간 호출이 되므로 런처가 계산해 넘긴다 (§5).
    V_tr = next((sg[3] for sg in k.MISSION_PROFILE if sg[1] == "trans"), 0.5 * k.V_cr)
    T_av, _, lim = prop.thrust_max(V_tr, pl.m_mot, pl.smot.kv, pre.pmap,
                                   pre.atm, pl.U_eval)
    sp_tr = prop.solve_point(V_tr, w.MTOW, pl.m_mot, pre.pmap, pre.aero, pre.atm,
                             pl.U_eval, kv=pl.smot.kv, pod=pod)
    dT_rotor = max(T_av - sp_tr.T, 0.0) / k.N_rot
    r.diag.update({"V_trans": V_tr, "T_avail_trans": T_av, "T_req_trans": sp_tr.T,
                   "dT_rotor": dT_rotor, "thrust_limit": lim})

    r.stab = sb = stab.run(dv, pre.hull, pre.aero, mp, lay, dT_rotor)
    r.g["g8"], r.g["g9"] = sb.g8, sb.g9
    r.diag.update({"SM": sb.SM, "M_dist": sb.M_dist, "M_ctrl": sb.M_ctrl,
                   "alpha_max": sb.alpha_max, "x_cg": mp.x_cg,
                   "J_yy": mp.J_yy, "x_cp": pre.aero.x_cp})

    # 연속 출력은 dash 축동력(로터 기수 합)을 쓴다 — 모터 단가의 기준
    r.cost = ct = cost.run(pl.m_mot, k.N_rot * pl.smot.P_shaft_dash, pl.E_batt,
                           dv.d_prop, pl.reqE.I_max, pl.strc.m_print)

    # ══ 성적표 (§6.2) ══
    r.ec = {
        "C1_MTOW[kg]": w.MTOW,
        "C2_R_dash[m]": rng.R_dash,
        "C3_margin_V": ev.margin_V,
        "C4_n_design": dv.n_design,
        "C5_alpha_max[rad/s2]": sb.alpha_max,
        "C6_SPL_hover[dB]": ev.SPL_hover,
        "C7_Cost_acq[KRW]": ct.Cost_acq,
        "C8_l_body[m]": pre.hull.l_body,
    }
    r.feasible = all(v >= 0 for v in r.g.values())
    if not r.feasible:
        # 품질 조건(g5–g9) 위반은 조기 탈락이 아니라 '불합격 기록'이다.
        r.fail_code = "quality:" + ",".join(n for n, v in r.g.items() if v < 0)
        r.fail_stage = "② 설계 품질 조건"
    return r


# ══════════════════════════════════════════════════════════════════════════
# 보고
# ══════════════════════════════════════════════════════════════════════════
_G_NAME = {"g1": "순항 성립", "g2": "팁 마하", "g3": "열 한계", "g4": "사이징 수렴",
           "g5": "구조 응력", "g6": "부품 내장", "g7": "프롭 클리어런스",
           "g8": "직진 안정", "g9": "천이 조종"}


def report(r: Result) -> None:
    stdout_utf8()
    print("=" * 64)
    print("합격 조건 (음수 = 불합격)")
    for name in [f"g{i}" for i in range(1, 10)]:
        if name in r.g:
            v = r.g[name]
            print(f"  {name}  {_G_NAME[name]:<12s} {v:+10.3f}  {'OK' if v >= 0 else 'FAIL'}")
        else:
            print(f"  {name}  {_G_NAME[name]:<12s} {'—':>10s}  (미계산 — 앞에서 탈락)")

    print("-" * 64)
    print("성적표 (EC)")
    if r.ec:
        for name, v in r.ec.items():
            print(f"  {name:<22s} {v:14.4f}")
    else:
        for name in ["C1_MTOW[kg]", "C2_R_dash[m]", "C3_margin_V", "C4_n_design",
                     "C5_alpha_max[rad/s2]", "C6_SPL_hover[dB]",
                     "C7_Cost_acq[KRW]", "C8_l_body[m]"]:
            print(f"  {name:<22s} {'—':>14s}  (미계산)")

    print("-" * 64)
    print(f"feasible   : {r.feasible}")
    print(f"fail_code  : {r.fail_code or '(없음)'}")
    print(f"fail_stage : {r.fail_stage or '(없음)'}")
    if r.diag:
        print("-" * 64)
        print("진단 변수")
        for name, v in r.diag.items():
            print(f"  {name:<16s} {v}")
    print("=" * 64)
    print("⚠ 계수 미확정 구간이 남아 있다 — MISS(C2)·GEOM 배치(g6·g7)·STRC(g5)")
    print("  ·STAB(C5,g8·g9)·COST(C7)·PROP.evaluate(C3·C6). 무엇이 가짜인지는 STUBS.md.")


if __name__ == "__main__":
    # 대표 설계점 — 범위가 전부 TBD 라 이 숫자들도 스모크 테스트용이다.
    dv = DesignVars(
        d_body=0.09, lambda_body=8.0, S_fin=0.030, x_fin=0.60, AR_fin=2.2,
        f_mount=1.0, n_design=4.0, d_prop=0.13, pd_prop=1.50, n_ser=6,
        k_E=1.0, k_mot=1.0,
    )   # g1~g9 를 전부 통과하는 설계점.
        # ⚠ STRC 담당 코드 이식 후 갱신했다. 새 구조 모델은 포드 쉘과 핀 외피/코어
        #   분리를 포함해 659 g → 799 g 로 무겁고, 늘어난 질량이 핀·포드(x≈0.59)에
        #   붙어 x_cg 가 후퇴한다. 이전 점(S_fin 0.036 · x_fin 0.55)은 SM 0.917 →
        #   0.839 로 떨어져 g8 불합격이 됐다. 구조 모델이 바뀐 결과지 이식 버그가 아니다.
        # g8·g9 는 서로 반대로 움직이므로(docs §11-36) 합격 밴드가 좁은 대각선이다.
        #   x_fin 0.58~0.60 밖은 한쪽이 깨진다.
        # pd_prop 은 §2 하한 규칙(prop.pd_prop_min ≈ 1.466)을 만족해야 한다
    report(evaluate(dv))
