"""PROP — 프롭 맵 · 작동점 · 모터 사이징 · 평가.  ICD0-008 §5.1

함수 다섯 개 중 solve_point 이 나머지가 공유하는 심장이다 (§4.6 커널 공유).
셀 개로전압 U_ocv(SOC) 도 전기 계통이라 여기 둔다 (§5 ▶부품 물성은 어디에).

모듈 간 직접 호출의 **유일한 예외**가 여기다 — size_motor 가 THRM 을 직접 부른다.

[좌표 규약 — §4.1 트림 연립의 전제, P3 에서 확정]
  θ : 자세각. **수평 기준**이다. θ=90° 호버(기수 수직), θ→0 순항(기체 수평).
      ICD §4.1 본문의 "호버(θ ≈ 90°)" 와 같은 기준이다.
  α : 받음각. 순항 비행경로각 γ=0(수평비행) 전제이므로 **α = θ** 다.
  평형 2식 (미지수 T, θ):
        T·sinθ + L(V, α) = W        (수직)
        T·cosθ           = D(V, α)  (수평)
  L=0 이면 T=√(D²+W²), tanθ=W/D 로 환원된다. L>0 이면 T 가 그보다 **작아진다** —
  이것이 §4.1 이 얻으려던 동체 양력 크레딧이다.

⚠ ICD §4.1 에 적힌 식은 `T·cosθ + L·sinθ = W`, `T·sinθ − D·cosθ = 0` 인데,
  어떤 좌표 규약으로도 닫히지 않는다 (풍축 L·D 로 보면 여분의 sinθ·cosθ 가 남고,
  동체축 N·A 로 보면 같은 힘이 두 식에 함께 나와야 한다). 위 형태로 바로잡았다.
  [확정 필요 — docs/ICD0-008.md §11-11]

[스텁] size_motor(P4) · evaluate(P3 이후) 는 아직 스텁이다.
"""
import math

import constants as k
from interfaces import (DesignVars, HullOut, AtmOut, AeroOut, PropMapOut,
                        MotorElecOut, SolvePointOut, SizeMotorOut, EvaluateOut)
from modules import thrm     # ← 유일하게 허용된 모듈 간 직접 호출 (§5)


# ══════════════════════════════════════════════════════════════════════════
# 부품 물성 — 셀 개로전압 · 팩 저항 · 평가 전압
# ══════════════════════════════════════════════════════════════════════════
def U_ocv(SOC: float) -> float:
    """셀당 개로전압 [V] — 3점 선형 (만충·공칭·종지).

    회귀가 아니라 상수 3개를 잇는 정의식이라 스텁이 아니다.
    상수 자체는 constants.py 에서 TBD 로 표시돼 있다.
    """
    SOC = min(max(SOC, 0.0), 1.0)
    if SOC >= 0.5:
        return k.U_cell_nom + (k.U_cell_full - k.U_cell_nom) * (SOC - 0.5) / 0.5
    return k.U_cell_cut + (k.U_cell_nom - k.U_cell_cut) * SOC / 0.5


def R_pack(E_batt: float, n_ser: int) -> float:
    """팩 내부저항 [Ω] — R = k_Rpack · n_ser / cap  (§3.2)."""
    cap_Ah = E_batt / max(U_ocv(1.0) * n_ser, 1e-9)     # Wh → Ah 환산
    return k.k_Rpack * n_ser / max(cap_Ah, 1e-9)


def U_eval(E_batt: float, n_ser: int, I_dash: float) -> float:
    """평가 전압 [V] — 방전 말기 최악 상태 (§4.5).

    U_eval = U_cell(1−DoD)·n_ser − I_dash·R_pack
    """
    return U_ocv(1.0 - k.DoD) * n_ser - I_dash * R_pack(E_batt, n_ser)


# ══════════════════════════════════════════════════════════════════════════
# 팁 마하 한계
# ══════════════════════════════════════════════════════════════════════════
def n_tip_limit(V: float, d_prop: float, a_snd: float) -> float:
    """나선 팁 마하 ≤ M_tip_max 를 만족하는 최대 회전수 [rev/s].

    나선 팁속도는 전진속도와 회전속도의 벡터합이다 — 전진할수록 회전 여유가 준다.
    """
    V_tip_allow = k.M_tip_max * a_snd
    v_rot_sq = V_tip_allow ** 2 - V ** 2
    if v_rot_sq <= 0.0:
        return 0.0          # 전진속도만으로 이미 한계 — 회전 여유가 없다
    return math.sqrt(v_rot_sq) / (math.pi * d_prop)


def M_tip(V: float, n: float, d_prop: float, a_snd: float) -> float:
    """나선 팁 마하수."""
    return math.hypot(V, math.pi * d_prop * n) / a_snd


def J_cruise(V: float, a_snd: float) -> float:
    """팁 마하 한계에서 돌 때의 전진비 — **d_prop 에 무관하다**.

        J = V/(n_max·d) = π·V / √((M_tip_max·a)² − V²)

    n_max = v_rot/(πd) 라 지름이 약분된다. 즉 순항 작동 전진비는 속도와 팁 마하
    한계만의 함수이고, 프롭을 키워도 J 는 그대로다 (가용추력만 d² 로 는다).
    """
    v_rot_sq = (k.M_tip_max * a_snd) ** 2 - V ** 2
    if v_rot_sq <= 0.0:
        return float('inf')
    return math.pi * V / math.sqrt(v_rot_sq)


def pd_prop_min(V: float, a_snd: float) -> float:
    """DOE 표본을 뿌릴 `pd_prop` 하한 — ICD §2 하한 규칙 (계수 P3 개정).

        V_pitch ≥ k_pitch_margin · V   ⟺   pd_prop ≥ k_pitch_margin · J_cruise(V)

    등피치 프롭은 추력 0 전진비 J₀ ≈ pd_prop 이므로, 이 규칙은 "작동 전진비를
    추력 0 지점에서 얼마나 떼어 놓을 것인가" 와 같은 말이다.
    g1(§6.1)은 ICD 정의(피치속도 ≥ 순항속도) 그대로다 — 이건 판정이 아니라
    표본 범위 규칙이라 판정을 흔들지 않는다.
    """
    return k.k_pitch_margin * J_cruise(V, a_snd)


# ══════════════════════════════════════════════════════════════════════════
# ⓪ 프롭 성능 맵 — BEMT
# ══════════════════════════════════════════════════════════════════════════
def _bemt(J: float, d_prop: float, pd_prop: float) -> tuple:
    """단일 전진비 J 에서의 (CT, CP). 블레이드 요소 + 운동량 이론.

    무차원 계수는 프롭 기하만의 함수라 n·ρ 에 무관하다 → n=1, ρ=1 로 푼다.
    익형 모델에 Re·마하 의존을 넣지 않았으므로 이 무관성이 정확히 성립한다.
    팁 마하는 성능 보정이 아니라 g2 한계로 따로 판정한다 (§6.1).

    각 요소에서 미지수는 유도속도 (v_a, v_t) 두 개다. 운동량 관계를 v_a 의
    2차식으로 풀어 J=0(호버)에서도 특이점이 생기지 않게 한다 —
    유도인자(a, a') 형태는 V=0 에서 0/0 이 된다.
    """
    D = d_prop
    R = 0.5 * D
    n = 1.0                         # rev/s — 무차원이라 임의
    Omega = 2.0 * math.pi * n
    V = J * n * D
    pitch = pd_prop * D             # [m/rev] — 피치비의 정의
    c = k.c_over_D * D              # 등코드 전제
    rho = 1.0                       # 무차원

    r_hub = k.r_hub_ratio * R
    dr = (R - r_hub) / k.n_bemt_elem
    V_tip = Omega * R

    T = 0.0
    Q = 0.0
    for i in range(k.n_bemt_elem):
        r = r_hub + (i + 0.5) * dr
        # 헬리컬(등피치) 블레이드의 기하 피치각 — 피치비가 정의하는 그 형상
        beta = math.atan2(pitch, 2.0 * math.pi * r)

        v_a = 0.0
        v_t = 0.0
        dT_be = dQ_be = 0.0
        for _ in range(k.n_bemt_iter):
            U_a = V + v_a
            U_t = Omega * r - v_t
            if U_t <= 0.0:          # 선회 유도가 회전을 삼켰다 — 해가 없다
                break
            phi = math.atan2(U_a, U_t)
            W_sq = U_a * U_a + U_t * U_t
            a_eff = beta - phi
            cl = min(max(k.cl_alpha_2d * a_eff, -k.cl_max_2d), k.cl_max_2d)
            cd = k.cd0_2d + k.k_cd_2d * cl * cl

            # Prandtl 팁 손실 — 유한 블레이드의 팁 하중 감소
            s_phi = max(abs(math.sin(phi)), 1e-9)
            f = 0.5 * k.B_blade * (R - r) / (r * s_phi)
            F = max((2.0 / math.pi) * math.acos(min(1.0, math.exp(-f))), 1e-4)

            # 블레이드 요소 하중 (단위 반경당)
            dT_be = 0.5 * rho * W_sq * k.B_blade * c * (cl * math.cos(phi)
                                                        - cd * math.sin(phi))
            dQ_be = 0.5 * rho * W_sq * k.B_blade * c * (cl * math.sin(phi)
                                                        + cd * math.cos(phi)) * r

            # 운동량 관계로 유도속도 갱신
            #   축 :  dT = 4π r ρ (V + v_a) v_a F   →  v_a² + V·v_a − dT/(4πrρF) = 0
            A_ax = 4.0 * math.pi * r * rho * F
            if dT_be > 0.0:
                disc = V * V + 4.0 * dT_be / A_ax
                v_a_new = 0.5 * (-V + math.sqrt(max(disc, 0.0)))
            else:
                v_a_new = 0.0
            #   접선 :  dQ = 4π r² ρ (V + v_a) v_t F
            den = 4.0 * math.pi * r * r * rho * (V + v_a_new) * F
            v_t_new = dQ_be / den if den > 1e-12 else 0.0

            d = abs(v_a_new - v_a) + abs(v_t_new - v_t)
            v_a += k.relax_bemt * (v_a_new - v_a)
            v_t += k.relax_bemt * (v_t_new - v_t)
            if d < k.eps_bemt * max(V_tip, 1.0):
                break

        T += dT_be * dr
        Q += dQ_be * dr

    P = Omega * Q
    CT = T / (rho * n ** 2 * D ** 4)
    CP = P / (rho * n ** 3 * D ** 5)
    return CT, CP


def _interp(xs: list, ys: list, x: float) -> float:
    """등간격 격자 선형 보간 — 격자 밖은 끝값 유지 (외삽 금지)."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    h = xs[1] - xs[0]
    i = int((x - xs[0]) / h)
    i = min(i, len(xs) - 2)
    w = (x - xs[i]) / h
    return ys[i] * (1.0 - w) + ys[i + 1] * w


def build_map(dv: DesignVars, air: AtmOut) -> PropMapOut:
    """⓪ BEMT 로 CT(J)·CP(J) 테이블을 미리 풀어 둔다.

    무차원 계수는 프롭 기하만의 함수라 모터·무게와 무관하다 → 루프 밖.

    [앵커 보정 없음] ICD §8 A-5 는 BEMT 가 J≈0 에서 추력을 크게 예측한다고
    경고하는데, 실측 데이터가 없는 상태에서 감쇠 함수 형태를 정하면 근거 없는
    숫자가 들어간다. 지금은 보정을 넣지 않는다 — **호버 추력이 과대평가된다.**
    """
    Js = [k.J_map_max * i / (k.n_J_grid - 1) for i in range(k.n_J_grid)]
    CTs, CPs = [], []
    for J in Js:
        ct, cp = _bemt(J, dv.d_prop, dv.pd_prop)
        CTs.append(ct)
        CPs.append(cp)

    def CT(J: float) -> float:
        return _interp(Js, CTs, J)

    def CP(J: float) -> float:
        return _interp(Js, CPs, J)

    # 프롭 질량은 정의식 m = k_mprop·d³ (§5.1). 계수만 TBD.
    m_prop = k.k_mprop * dv.d_prop ** 3

    # g1 — 피치속도(= pitch × 허용 최대 rpm)가 V_cr 이상인지.
    # 허용 최대 rpm 은 순항속도에서의 나선 팁 마하 한계에서 나온다.
    n_max_cr = n_tip_limit(k.V_cr, dv.d_prop, air.a_snd)
    V_pitch = dv.pd_prop * dv.d_prop * n_max_cr
    g1 = V_pitch - k.V_cr

    return PropMapOut(CT=CT, CP=CP, m_prop=m_prop, g1=g1, V_pitch=V_pitch,
                      d_prop=dv.d_prop)


# ══════════════════════════════════════════════════════════════════════════
# 내부 — DC 모터 모델
# ══════════════════════════════════════════════════════════════════════════
def motor_regression(m_mot: float, kv: float) -> tuple:
    """모터 질량·kv → (R_mot [Ω], I0 [A]).  ICD §8 A-2 회귀.

    [스텁] 회귀 계수가 아직 없다. constants.py 의 TBD 계수를 그대로 쓴다 —
           형태(멱함수)까지 잠정이며 스펙표 3~5종을 모으면 바뀔 수 있다.
    """
    R_mot = k.a_R * m_mot ** k.b_R * kv ** k.c_R    # [스텁] 계수 TBD
    I0 = k.a_I0 * kv ** k.b_I0                      # [스텁] 계수 TBD
    return R_mot, I0


def motor_elec(tau: float, omega: float, kv: float,
               R_mot: float, I0: float, U_bus: float) -> MotorElecOut:
    """고전 DC 모터 모델 (§4.3). 상수 효율 가정 금지.

        K_t = 60/(2π·K_v) ,  I = τ/K_t + I0 ,  U = I·R_mot + ω/K_v ,  P_cu = I²·R_mot

    수식 자체는 확정이고, R_mot·I0 를 주는 회귀 계수가 TBD 다 (§8 A-2).
    """
    Kv_rad = kv * 2.0 * math.pi / 60.0      # [rad/s/V] — kv[rpm/V] 환산
    K_t = 1.0 / Kv_rad                      # [N·m/A] = 60/(2π·kv)
    I = tau / K_t + I0
    U_req = I * R_mot + omega / Kv_rad
    return MotorElecOut(I=I, U_req=U_req, P_cu=I * I * R_mot)


def _solve_kv(tau: float, omega: float, U_bus: float, m_mot: float) -> tuple:
    """요구 (τ, ω) 를 버스 전압 U_bus 로 내는 kv 를 찾는다 — 결정론적 이분법.

    ICD §5.1: "kv 는 이 토크 평형의 해로 나옵니다. 설계변수가 아닙니다."
    R_mot·I0 가 kv 의 함수라 해석해가 없으므로 근찾기로 닫는다.
    반환: (kv, MotorElecOut) 또는 실패 시 (None, None)
    """
    def resid(kv):
        R_mot, I0 = motor_regression(m_mot, kv)
        me = motor_elec(tau, omega, kv, R_mot, I0, U_bus)
        return me.U_req - U_bus, me

    lo, hi = k.kv_lo, k.kv_hi
    f_lo, _ = resid(lo)
    f_hi, _ = resid(hi)
    if f_lo * f_hi > 0.0:
        return None, None                   # 구간 안에 해가 없다
    for _ in range(k.N_bisect_max):
        mid = 0.5 * (lo + hi)
        f_mid, me = resid(mid)
        if abs(f_mid) < 1e-12 * max(U_bus, 1.0):
            return mid, me
        if f_lo * f_mid <= 0.0:
            hi = mid
        else:
            lo, f_lo = mid, f_mid
    mid = 0.5 * (lo + hi)
    _, me = resid(mid)
    return mid, me


# ══════════════════════════════════════════════════════════════════════════
# 공용 — 작동점 풀이
# ══════════════════════════════════════════════════════════════════════════
def _trim_init(V: float, W: float, qS: float, aer: AeroOut, pod) -> tuple:
    """트림 초기값 규칙 — 결정론적 2단 (P3 확정).

    1. **양력지지 해**가 있으면 그것을 쓴다. qS·CL(V, α) = W 가 되는 α 를 찾아
       θ₀=α, T₀=D(V,α)/cosθ₀. 순항·dash 의 실제 해가 이 근처다.
    2. 없으면(양력이 무게를 못 든다) **무양력 해** θ₀=atan2(W, D₀), T₀=√(W²+D₀²).
       저속·과중량 구간이 여기다.

    무양력 해만 쓰면 순항에서 θ₀ 가 실제 해(~1°)와 수십 도 떨어져 뉴턴이 헤맨다.

    CL(α) 는 단조가 아니다 — 큰 α 에서 축력항 −CD0·sinα 가 이겨 다시 내려온다.
    그래서 이분법을 바로 걸지 않고 고정 격자를 훑어 **첫 교차 구간**을 잡은 뒤
    그 안에서만 이분법을 돈다. 격자가 고정이라 결정론적이다.
    """
    D0 = aer.F_drag(V, 0.0, pod)
    N = 90
    prev_a, prev_f = 0.0, qS * aer.CL(V, 0.0, pod) - W
    for i in range(1, N + 1):
        a = k.theta_hi * i / N
        f = qS * aer.CL(V, a, pod) - W
        if prev_f < 0.0 <= f:               # 첫 교차 구간을 찾았다
            lo, hi = prev_a, a
            for _ in range(k.N_bisect_max):
                mid = 0.5 * (lo + hi)
                if qS * aer.CL(V, mid, pod) - W < 0.0:
                    lo = mid
                else:
                    hi = mid
            th0 = 0.5 * (lo + hi)
            return max(aer.F_drag(V, th0, pod) / max(math.cos(th0), 1e-6), 1e-9), th0
        prev_a, prev_f = a, f
    return math.hypot(W, D0), math.atan2(W, max(D0, 1e-9))


def _trim(V: float, W: float, aer: AeroOut, air: AtmOut, pod=None) -> tuple:
    """§4.1 순항 트림 — 미지수 (T, θ) 에 대한 2×2 뉴턴.

        r1 = T·sinθ + q·S_ref·CL(V, θ) − W
        r2 = T·cosθ − F_drag(V, θ)

    비행경로각 γ=0 전제이므로 α = θ 다 (P3 확정).
    스텝은 잔차 노름이 줄 때까지 반으로 접는다(백트래킹) — CN_α 가 커서 순수 뉴턴은
    자세각을 크게 던지고 되돌아오길 반복한다.
    반환: (T, theta, n_iter, ok)
    """
    qS = 0.5 * air.rho * V * V * aer.S_ref      # CL 을 힘으로 바꾸는 스케일

    def resid(T, th):
        r1 = T * math.sin(th) + qS * aer.CL(V, th, pod) - W
        r2 = T * math.cos(th) - aer.F_drag(V, th, pod)
        return r1, r2

    def norm(T, th):
        r1, r2 = resid(T, th)
        return math.hypot(r1, r2)

    T, th = _trim_init(V, W, qS, aer, pod)

    h_th = 1e-7
    for it in range(1, k.N_trim_max + 1):
        r1, r2 = resid(T, th)
        f0 = math.hypot(r1, r2)
        if f0 < k.eps_trim * max(W, 1.0):
            return T, th, it, True

        # 수치 야코비안 — CL·F_drag 가 클로저라 해석 미분이 없다.
        # T 에 대한 편미분은 해석적으로 안다: ∂r1/∂T=sinθ, ∂r2/∂T=cosθ.
        a11, a21 = math.sin(th), math.cos(th)
        p1, p2 = resid(T, th + h_th)
        m1, m2 = resid(T, th - h_th)
        a12 = (p1 - m1) / (2 * h_th)
        a22 = (p2 - m2) / (2 * h_th)

        det = a11 * a22 - a12 * a21
        if abs(det) < 1e-30:
            return T, th, it, False
        dT = (-r1 * a22 + r2 * a12) / det
        dth = (-a11 * r2 + a21 * r1) / det
        dth = min(max(dth, -k.dtheta_max), k.dtheta_max)

        # 백트래킹 — 잔차가 줄어드는 스텝만 받는다
        step = 1.0
        for _ in range(20):
            T_n = max(T + step * dT, 1e-9)
            th_n = min(max(th + step * dth, k.theta_lo), k.theta_hi)
            if norm(T_n, th_n) < f0:
                break
            step *= 0.5
        else:
            return T, th, it, False     # 어떤 스텝도 잔차를 못 줄인다
        T, th = T_n, th_n

    return T, th, k.N_trim_max, False


def _rpm_for_thrust(T_1: float, V: float, pmap: PropMapOut, air: AtmOut,
                    d_prop: float) -> tuple:
    """로터 1기 요구추력 T_1 을 내는 회전수 [rev/s] — 1차원 이분법.

    T(n) = CT(V/(nD))·ρ·n²·D⁴ 는 n 에 단조증가라 이분법이 반드시 수렴한다.
    상한은 나선 팁 마하 한계다 — 그 위는 g2 위반이라 애초에 못 쓴다.
    반환: (n, n_iter, ok)
    """
    D = d_prop
    n_hi = n_tip_limit(V, D, air.a_snd)
    if n_hi <= 0.0:
        return 0.0, 0, False

    def thrust(n):
        if n <= 0.0:
            return 0.0
        J = V / (n * D)
        return pmap.CT(J) * air.rho * n * n * D ** 4

    if thrust(n_hi) < T_1:
        return n_hi, 0, False       # 팁 마하 한계 안에서 낼 수 없는 추력

    lo, hi = 0.0, n_hi
    it = 0
    for it in range(1, k.N_bisect_max + 1):
        mid = 0.5 * (lo + hi)
        if thrust(mid) < T_1:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-9 * max(n_hi, 1.0):
            break
    return 0.5 * (lo + hi), it, True


def solve_point(V: float, MTOW: float, m_mot: float,
                pmap: PropMapOut, aer: AeroOut, air: AtmOut,
                U_bus: float, hover: bool = False, pod=None) -> SolvePointOut:
    """공용 작동점 — 평형이 두 겹이다 (§5.1 PROP).

      1. 기체 트림   : 미지수 (T, θ) 에 대한 2×2 뉴턴 (§4.1)
      2. 파워트레인 : 요구추력을 내는 rpm 을 1차원 이분법으로 찾고,
                      그 (τ, ω) 를 버스 전압으로 내는 kv 를 다시 이분법으로 찾는다

    호버(θ=90°)는 1번을 건너뛰고 추력지지(T=W)로 푼다.
    kv 는 토크 평형의 **해**로 나온다 — 설계변수가 아니다.

    두 겹은 **중첩**으로 푼다 (P3 확정): 바깥이 트림, 안쪽이 rpm.
    안쪽이 단조함수라 이분법이 반드시 수렴하고, 바깥 2×2 는 야코비안이 작아 안정적이다.
    수렴 실패는 infeasible 이다 — ok=False 로 알리고 호출부가 사유 코드로 옮긴다.
    """
    W = MTOW * k.g

    if hover:
        T = W
        theta = math.pi / 2.0
        n_trim = 0
        trim_ok = True
    else:
        T, theta, n_trim, trim_ok = _trim(V, W, aer, air, pod)

    if not trim_ok:
        return SolvePointOut(T=T, theta=theta, rpm=0.0, I=0.0, P=0.0,
                             kv=0.0, P_cu=0.0, ok=False)

    # ── 파워트레인 ──
    T_1 = T / k.N_rot
    D = pmap.d_prop
    n, n_bis, rpm_ok = _rpm_for_thrust(T_1, V, pmap, air, D)
    if not rpm_ok:
        return SolvePointOut(T=T, theta=theta, rpm=n * 60.0, I=0.0, P=0.0,
                             kv=0.0, P_cu=0.0, ok=False)

    J = V / (n * D) if n > 0 else 0.0
    P_shaft = pmap.CP(J) * air.rho * n ** 3 * D ** 5
    omega = 2.0 * math.pi * n
    tau = P_shaft / omega if omega > 0 else 0.0

    kv, me = _solve_kv(tau, omega, U_bus, m_mot)
    if kv is None:
        return SolvePointOut(T=T, theta=theta, rpm=n * 60.0, I=0.0, P=0.0,
                             kv=0.0, P_cu=0.0, ok=False)

    # 전기 소요 — 로터 기수만큼, ESC 효율로 나눈다
    P_elec = k.N_rot * U_bus * me.I / k.eta_esc
    I_pack = P_elec / max(U_bus, 1e-9)

    return SolvePointOut(T=T, theta=theta, rpm=n * 60.0, I=I_pack, P=P_elec,
                         kv=kv, P_cu=k.N_rot * me.P_cu, ok=True)


# ══════════════════════════════════════════════════════════════════════════
# ① 모터 사이징
# ══════════════════════════════════════════════════════════════════════════
def size_motor(MTOW: float, pmap: PropMapOut, aer: AeroOut, air: AtmOut,
               U_bus: float, k_mot: float) -> SizeMotorOut:
    """① 요구 → 모터 질량.  m_mot 에 대한 **결정론적 이분법**.

    [스텁] P4 에서 구현한다. solve_point 는 이제 준비돼 있다.
    """
    return SizeMotorOut(
        m_mot=0.0,          # [스텁] 실제 모터 질량 아님
        I_dash=0.0,         # [스텁]
        g2=0.0,             # [스텁] 실제 팁 마하 판정 아님
        g3=0.0,             # [스텁] 실제 열 판정 아님
        active="stub",      # [스텁] cruise/hover 판별 미구현
        n_bisect=0,
    )


# ══════════════════════════════════════════════════════════════════════════
# ② 성능 평가
# ══════════════════════════════════════════════════════════════════════════
def evaluate(MTOW: float, m_mot: float, E_batt: float, n_ser: int,
             pmap: PropMapOut, aer: AeroOut, air: AtmOut,
             U_bus: float) -> EvaluateOut:
    """② 최고속도 실계산 + 소음.  [스텁] P3 이후 구현한다."""
    return EvaluateOut(
        margin_V=0.0,       # [스텁] ← EC C3 (가중치 33.7%)
        SPL_hover=0.0,      # [스텁] ← EC C6
        kv=0.0,             # [스텁]
        P_hover=0.0,        # [스텁]
        V_max=0.0,          # [스텁]
    )
