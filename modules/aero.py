"""AERO — 항력 빌드업 · 안정성 계수.  ICD0-008 §5.1

종속성: 무게·부품과 무관 → ⓪에서 1회만 실행하고, 반복 안에서는 곡선을 조회만 한다.

[구조 완료 — 부분 검증]
성분별 빌드업·압축성 보정·Barrowman 은 표준식대로 구현했다. 다만 형상 계수
(k_base·k_int·k_cal·k_cp_nose·크로스플로 4종)가 전부 TBD 라, 값의 절대 정확도는
앵커 실측 전까지 보증되지 않는다. 구조와 반응 방향만 믿을 수 있다.

[포드 항력 — ICD0-008 대비 로컬 개정]
ICD §5.1 은 포드(모터 나셀) 항력을 성분 목록에서 뺐다. 포드 치수가 모터 질량의 함수라
⓪에서 알 수 없기 때문인데, 그렇다고 빼 버리면 4기 나셀의 마찰·형상 항력이 통째로
사라져 항력→동력→배터리→무게 사슬이 계통적으로 과소평가된다.

그래서 **식은 AERO 가 소유하되 포드 치수는 인자로 받는다.** 곡선 자체는 여전히
설계변수만의 함수라 ⓪가 MTOW 를 되먹임 참조하지 않는다 (§9 위반 아님).
포드 치수를 아는 시점(① size_motor 이후)에 호출부가 pod=(d_pod, l_pod) 를 넘긴다.
pod=None 이면 포드 항이 0 이고, 그건 '아직 안 넘겼다'는 뜻이지 '포드가 없다'가 아니다.

[결정 필요] α 와 자세각 θ 의 관계 정의 — 순항 비행경로각 가정. P3 에서 확정한다 (§8 A-1).
"""
import math

import constants as k
from interfaces import DesignVars, HullOut, AtmOut, AeroOut


def _Cf(Re: float) -> float:
    """난류 평판 마찰계수 — Prandtl-Schlichting 상관식."""
    return 0.455 / (math.log10(max(Re, 1e4))) ** 2.58


def _FF_body(lambda_body: float) -> float:
    """축대칭 동체 형상계수 — Hoerner. 세장비가 클수록 1 에 수렴한다."""
    return 1.0 + 60.0 / lambda_body ** 3 + 0.0025 * lambda_body


def _FF_fin(tc: float) -> float:
    """익형 형상계수 — Hoerner. 두께비가 두꺼울수록 압력항력이 붙는다."""
    return 1.0 + 2.0 * tc + 60.0 * tc ** 4


def run(dv: DesignVars, hl: HullOut, air: AtmOut) -> AeroOut:
    """⓪ 성분별 항력 빌드업 + 압축성 보정 + Barrowman."""
    q_cr = 0.5 * air.rho * k.V_cr ** 2

    # ══ 압축성 보정 ══
    # Prandtl-Glauert. 마찰항은 마하 의존이 약해 그대로 두고, 압력에서 나오는 항
    # (기저·핀 압력·간섭)과 법선력 기울기에만 1/β 를 적용한다.
    def _beta_pg(V: float) -> float:
        M = min(V / air.a_snd, k.M_pg_max)      # β→0 발산을 상한으로 막는다
        return math.sqrt(1.0 - M * M)

    # ══ 성분별 항력 빌드업 (전부 S_ref 기준으로 환산) ══
    def CD0(V: float, pod=None) -> float:
        """pod : (d_pod[m], l_pod[m]) 또는 None. 위 [포드 항력] 주석 참조."""
        V = max(V, 1.0)                         # Re→0 에서 상관식이 무너진다
        Re_b = V * hl.l_body / air.nu
        Re_f = V * hl.c_root / air.nu

        # 동체 마찰 — 평판 상관식 × 형상계수 × 젖음면적비
        cd_body = _Cf(Re_b) * _FF_body(dv.lambda_body) * hl.S_wet_body / hl.S_ref
        # 핀 마찰·압력 — 양면이라 2×Cf
        cd_fin = 2.0 * _Cf(Re_f) * _FF_fin(k.tc_fin) * hl.S_wet_fin / hl.S_ref
        # 기저 항력 — 기저면적 = S_ref (무딘 기저)
        cd_base = k.k_base
        # 간섭 항력 — 접합부 수 × 두께²  (핀-동체 2면 × N_rot)
        cd_int = 2.0 * k.N_rot * k.k_int * hl.t_fin ** 2 / hl.S_ref

        # 포드 마찰·형상 — N_rot 기, 세장비는 넘겨받은 치수에서 직접 낸다
        cd_pod = 0.0
        if pod is not None:
            d_pod, l_pod = pod
            Re_p = V * l_pod / air.nu
            S_wet_pod = k.N_rot * math.pi * d_pod * l_pod * k.k_form
            FF_pod = 1.0 + 0.35 / max(l_pod / d_pod, 1e-9)      # Hoerner 회전체
            cd_pod = _Cf(Re_p) * FF_pod * S_wet_pod / hl.S_ref

        beta = _beta_pg(V)
        return cd_body + cd_pod + (cd_fin + cd_base + cd_int) / beta

    # ══ Barrowman — 법선력 기울기와 압력중심 ══
    # 노즈: 기저 지름이 d_body 인 어떤 노즈든 CN_α = 2 (S_ref 기준)
    CN_nose = k.CN_nose
    x_nose = k.k_cp_nose * hl.l_nose

    # 핀: 동체 간섭 계수 K_fb 를 곱한 아음속 핀 법선력
    K_fb = 1.0 + hl.r_body / (hl.b_fin + hl.r_body)
    l_mid = math.sqrt(hl.b_fin ** 2 + (hl.x_t + (hl.c_tip - hl.c_root) / 2.0) ** 2)
    CN_fin = (K_fb * 4.0 * k.N_rot * (hl.b_fin / dv.d_body) ** 2
              / (1.0 + math.sqrt(1.0 + (2.0 * l_mid / (hl.c_root + hl.c_tip)) ** 2)))

    x_fin_cp = (hl.x_fin
                + hl.x_t * (hl.c_root + 2.0 * hl.c_tip) / (3.0 * (hl.c_root + hl.c_tip))
                + (1.0 / 6.0) * ((hl.c_root + hl.c_tip)
                                 - hl.c_root * hl.c_tip / (hl.c_root + hl.c_tip)))

    # 압축성 보정은 노즈·핀에 같은 배율로 걸리므로 x_cp 는 변하지 않는다.
    beta_cr = _beta_pg(k.V_cr)
    CN_alpha = (CN_nose + CN_fin) / beta_cr
    CN_alpha_fin = CN_fin / beta_cr
    x_cp = (CN_nose * x_nose + CN_fin * x_fin_cp) / (CN_nose + CN_fin)

    # ══ 고받음각 — 평면형 투영과 크로스플로 ══
    A_nose = k.k_side * dv.d_body * hl.l_nose
    A_cyl = dv.d_body * hl.l_cyl
    A_fin = k.k_finproj * dv.S_fin
    A_plan = A_nose + A_cyl + A_fin

    def C_N(alpha: float) -> float:
        """법선력계수 — 포텐셜 항 + 크로스플로 항 (Allen-Perkins)."""
        pot = CN_alpha * math.sin(alpha) * math.cos(alpha)
        cross = k.eta_cf * k.Cd_c * (A_plan / hl.S_ref) * math.sin(alpha) ** 2
        return pot + cross

    CD0_cr = CD0(k.V_cr)

    # ══ 풍축 변환 — 트림 연립(§4.1)이 쓰는 양력·항력 ══
    # 법선력 N 과 축력 A 를 받음각으로 돌린다:  L = N·cosα − A·sinα,  D = N·sinα + A·cosα
    def CL(V: float, alpha: float, pod=None) -> float:
        """(V, α) → 양력계수 (S_ref 기준) → PROP 트림 연립.

        ICD §5.1 은 CL(α) 로 적었으나 축력항이 V 에 의존한다(마찰항의 Re·마하 효과).
        dash 구간에서 순항 CD0 로 고정하면 수 % 오차가 붙어 V 를 받도록 넓혔다.
        [로컬 개정 — ICD 등재 요청]
        """
        return C_N(alpha) * math.cos(alpha) - CD0(V, pod) * math.sin(alpha)

    def F_drag(V: float, alpha: float = 0.0, pod=None) -> float:
        """(V, α) → 항력 [N]. 받음각이 붙으면 법선력의 항력 성분이 더해진다."""
        CD = C_N(alpha) * math.sin(alpha) + CD0(V, pod) * math.cos(alpha)
        return k.k_cal * 0.5 * air.rho * V ** 2 * hl.S_ref * CD

    return AeroOut(
        F_drag=F_drag, CL=CL,
        CN_alpha=CN_alpha, x_cp=x_cp,
        CN_alpha_fin=CN_alpha_fin, q_cr=q_cr, CD0_cr=CD0_cr, S_ref=hl.S_ref,
    )


if __name__ == "__main__":   # 검산 — 물리적으로 말이 되는 범위인지 (TASKS P2 완료판정)
    from common.out import stdout_utf8
    from modules import atm, geom
    stdout_utf8()

    dv = DesignVars(d_body=0.09, lambda_body=7.0, S_fin=0.036, x_fin=0.50,
                    AR_fin=2.2, f_mount=0.8, n_design=4.0, d_prop=0.13,
                    pd_prop=1.30, n_ser=6, k_E=1.0, k_mot=1.0)
    air = atm.run(0.0)
    hl = geom.hull(dv)
    a = run(dv, hl, air)

    V = k.V_cr
    POD = (0.030, 0.045)      # 검산용 가정 치수 — ① size_motor 가 오면 실제 값이 온다

    print(f"CD0(V_cr)  = {a.CD0_cr:.4f}      (기대 범위 0.2–0.8)")
    print(f"CD0 + 포드 = {a.F_drag(V, 0.0, POD) / a.F_drag(V, 0.0) * a.CD0_cr:.4f}"
          f"      (포드 {POD[0]*1e3:.0f}×{POD[1]*1e3:.0f} mm 4기 가정)")
    print(f"CN_alpha   = {a.CN_alpha:.3f} /rad   (핀 성분 {a.CN_alpha_fin:.3f})")
    print(f"x_cp       = {a.x_cp:.4f} m  = {a.x_cp / hl.l_body:.3f}·l_body")
    print(f"F_drag(V_cr, 0) = {a.F_drag(V, 0.0):.3f} N"
          f"  (포드 포함 {a.F_drag(V, 0.0, POD):.3f} N)")
    print(f"CL(V_cr, 0°/5°/10°) = {a.CL(V, 0.0):.3f} / {a.CL(V, math.radians(5)):.3f}"
          f" / {a.CL(V, math.radians(10)):.3f}")
    print(f"CL 의 V 의존  = {a.CL(60.0, math.radians(10)):.4f} (60 m/s)"
          f" / {a.CL(120.0, math.radians(10)):.4f} (120 m/s)")

    assert 0.2 <= a.CD0_cr <= 0.8, a.CD0_cr
    assert a.CN_alpha > 0 and a.x_cp > 0
    assert abs(a.CL(V, 0.0)) < 1e-12, "α=0 에서 양력은 0 이어야 한다"
    assert a.CL(V, math.radians(10)) > a.CL(V, math.radians(5)) > 0, "CL 이 α 에 증가해야 한다"
    assert a.F_drag(V, math.radians(10)) > a.F_drag(V, 0.0), "받음각 항력 증가"
    assert a.F_drag(V, 0.0, POD) > a.F_drag(V, 0.0), "포드를 넘기면 항력이 커져야 한다"
    print("AERO 검산 통과")
