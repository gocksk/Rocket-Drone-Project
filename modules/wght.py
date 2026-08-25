"""WGHT — 무게 합산·수렴. [실제 구현 — 통합의 핵심]
ICD0-008 §5.1 WGHT · §5 ▶사이징 루프 상세
① 사이징 루프가 여기다. 되먹임은 MTOW 단 하나.

[ICD0-008 계약 일반화 — §8 C-3]
응답 질량이 구조 하나에서 구조+모터+배터리 셋으로 늘어났다. 그래서 주입 함수를
  strc_of(MTOW) -> StrcOut          (ICD0-007)
  resp_of(MTOW) -> (응답질량_합, payload)   (ICD0-008)
로 일반화했다. payload 는 루프가 **들여다보지 않는** 불투명 객체다.
수학·상태머신·판정 로직은 무변경이다 — 아래 _iterate 의 분기는 한 줄도 바뀌지 않았다.

[수렴을 '되나/안되나'가 아니라 '왜 안되나'로 나눈다 — §3]
반복이 안 끝나는 이유는 셋이고, 책임 소재가 서로 다르다.

  1. 고정점이 아예 없다       (Ŝ ≥ 1)         → STRC 모델 책임
  2. 고정점은 있는데 멀어진다  (r̂ ≤ -(1+δ))    → 우리의 beta 선택 책임
  3. 이산화 한계에서 진동한다  (r̂ ≈ -1, 유계)  → 한계까지 수렴한 것. 탈락시키면 안 된다

셋을 converged=False 하나로 뭉치면 3번 설계점이 조용히 OEC 후보에서 빠지고,
1번·2번을 구분 못 해 누가 고쳐야 하는지도 알 수 없다.

[완화(beta) 격리 — §3]
beta 는 갱신식과 Ŝ 환산식 두 줄에만 등장한다. 판정·가드 어디에도 beta 에 의존하는
양을 두지 않는다. 그래서 '완화된 스텝(Δ)'이라는 변수를 아예 만들지 않고 resid 만 쓴다.
"""
import constants as k
from interfaces import WghtOut, MassProps


def _options(over=None):
    """반복 옵션. 기본값은 전부 constants.py 에서 온다 (숫자 하드코딩 금지 규칙)."""
    o = {'eps_conv': k.eps_conv, 'beta': k.beta, 'N_iter_max': k.N_iter_max,
         'resid_floor': k.resid_floor, 'delta_r': k.delta_r,
         'n_confirm': k.n_confirm, 'n_min_iter': k.n_min_iter}
    o.update(over or {})
    return o


def _iterate(MTOW_0, m_fixed, resp_of, options=None):
    """MTOW 고정점 반복 — 순수 함수.

    MTOW_0   : 초기 추정값 [kg]
    m_fixed  : MTOW 와 무관한 질량 합 [kg]
    resp_of  : MTOW -> (응답질량_합[kg], payload) 인 순수 함수.
               dv/hull/aero/맵은 호출부가 닫아서 넘긴다. payload 는 불투명 객체라
               이 루프가 내용을 들여다보지 않는다.
    options  : _options() 참조

    이 함수가 dv/hull/aero 를 모르는 것이 핵심이다. 그래야 수렴 특성을
    가짜 응답 모델(정답을 아는 스텁)로 단독 검증할 수 있다 — validation/wght/ 참조.

    반환: {'MTOW', 'payload', 'status', 'S_hat', 'err', 'n_iter', 'history'}
      MTOW  : 마지막 STRC 호출의 합 (완화된 반복값이 아니다). Σbreakdown 과 항등
      status: converged / diverged_structural / diverged_numerical / limit_cycle / max_iter
      S_hat : 수렴점 근방의 dm_str/dMTOW. 추정 불가 시 None
      err   : 반환 MTOW 의 오차 상한. Ŝ ≥ 1 이면 None
    """
    o = _options(options)

    if o['N_iter_max'] < 1:
        raise ValueError(f"N_iter_max 는 1 이상이어야 합니다: {o['N_iter_max']}")

    # eps_conv 가 resid_floor 이하면 하한 가드가 수렴 판정을 항상 선점한다.
    # 판정은 |resid| < eps·(1-Ŝ)·MTOW 에서, 가드는 |resid| < resid_floor·MTOW 에서
    # 걸리므로 (1-Ŝ) < 1 인 이상 eps ≤ resid_floor 면 무조건 가드가 먼저다.
    # 그러면 status 는 converged 로 나오지만 Ŝ=None 이 되어 오차 보고가 불가능해진다.
    if o['eps_conv'] <= o['resid_floor']:
        raise ValueError(
            f"eps_conv({o['eps_conv']:.3e}) 가 resid_floor({o['resid_floor']:.3e}) 이하입니다. "
            "하한 가드가 항상 먼저 발동해 eps_conv 가 아무 역할도 못 합니다.")

    MTOW = MTOW_0
    pl = raw = resid = None
    status, S_hat, err = 'max_iter', None, None
    resid_prev, prev = None, None      # prev = (raw, resid, pl) — 리밋사이클 분기 선택용
    n_div_struct = n_div_osc = n_cycle = 0
    history = []
    it = 0

    for it in range(1, o['N_iter_max'] + 1):
        m_resp, pl = resp_of(MTOW)
        raw = m_fixed + m_resp         # 반환 대상. Σbreakdown 과 항등 (§4)
        resid = raw - MTOW             # 완화 전, 부호 유지
        history.append((MTOW, raw, resid))

        # ── (1) resid 하한 가드 ──
        # resid 가 수치 노이즈 수준으로 내려가면 r̂ 은 노이즈만 잰다. 노이즈는 방향이
        # 없어 Ŝ 가 1 근처에도 1 이상에도 찍힌다 — '발산 오판'과 '수렴 불능'은 같은
        # 현상의 양면이라 가드를 한 곳에 두고, 하한 미달은 '이미 수렴'으로 보낸다.
        if abs(resid) < o['resid_floor'] * MTOW:
            status, S_hat, err = 'converged', None, 0.0
            break

        if resid_prev is not None:
            # r̂ 을 '완화된 스텝의 비'가 아니라 'resid 의 비'로 정의한다.
            # 값은 같고(beta 가 약분된다) 완화 격리가 정의 단계에서 보장된다.
            r_hat = resid / resid_prev
            S_hat = 1.0 - (1.0 - r_hat) / o['beta']

            # 카운터는 r̂ 이 생긴 시점부터 세되, 판정은 n_min_iter 이후에만 한다.
            n_div_struct = n_div_struct + 1 if r_hat >= 1.0 else 0
            n_div_osc = n_div_osc + 1 if r_hat <= -(1.0 + o['delta_r']) else 0
            in_band = -(1.0 + o['delta_r']) <= r_hat <= -(1.0 - o['delta_r'])
            n_cycle = n_cycle + 1 if (in_band and abs(resid) <= abs(resid_prev)) else 0

            if it >= o['n_min_iter']:
                # ── (2) 구조 발산 ──
                # r̂ ≥ 1 ⇔ Ŝ ≥ 1 (beta > 0 에서 항상). deadband 를 두지 않는다:
                # 두면 1 < Ŝ < 1+δ 가 어느 status 에도 안 걸리는 구멍이 생기고,
                # 그 구간에서 아래 (5) 의 분모가 음수가 되어 판정식이 무조건 통과한다.
                if n_div_struct >= o['n_confirm']:
                    status = 'diverged_structural'
                    break

                # ── (3) 진동 발산 (수치) ──
                if n_div_osc >= o['n_confirm']:
                    status = 'diverged_numerical'
                    break

                # ── (4) 리밋 사이클 ──
                # 슬라이서 출력이 계단형(레이어 수·둘레 수가 정수)이면 MTOW 가 두 값
                # 사이를 무한 진동하고 |resid| 는 양자 크기에서 멈춘다. 발산이 아니다.
                # 중점을 반환하면 안 된다 — 어떤 STRC 호출 결과와도 같지 않아 §4 의
                # 항등식이 깨지고, print_setting 의 레이어 수는 정수라 평균이 물리적으로
                # 존재하지도 않는다. 그래서 |resid| 작은 쪽 '분기를 통째로' 채택하고,
                # '참값이 두 값 사이'라는 의도는 값이 아니라 err 로 표현한다.
                if n_cycle >= o['n_confirm'] and prev is not None:
                    err = abs(raw - prev[0]) / 2.0        # 양자/2
                    if abs(prev[1]) < abs(resid):
                        raw, pl = prev[0], prev[2]
                    status = 'limit_cycle'
                    break

                # ── (5) 수렴 ──
                # 스텝이 아니라 '오차'로 판정한다. 남은 오차는 등비급수 합이라
                # |resid|/(1-Ŝ) 이고, Ŝ 가 1 에 가까울수록 같은 스텝이라도 실제
                # 오차는 커진다. |resid| 만 보면 그만큼 과소평가한다.
                # Ŝ<1 을 전제로 명시한 이유는 (2) 주석 참조.
                if S_hat < 1.0 and abs(resid) / (1.0 - S_hat) < o['eps_conv'] * MTOW:
                    status = 'converged'
                    err = S_hat * abs(resid) / (1.0 - S_hat)
                    break

        resid_prev = resid
        prev = (raw, resid, pl)
        MTOW = MTOW + o['beta'] * resid        # 갱신 — beta 는 여기서만 쓴다
    else:
        # ── (6) 반복 소진 ──
        # 이진 판단을 호출부에 떠넘기지 않는다. err 을 같이 줘서 임계로 처리하게 한다.
        if S_hat is not None and S_hat < 1.0:
            err = abs(S_hat) * abs(resid) / (1.0 - S_hat)

    return {'MTOW': raw, 'payload': pl, 'status': status, 'S_hat': S_hat,
            'err': None if err is None else abs(err),
            'n_iter': it, 'history': history}


# limit_cycle 은 '이산화 한계까지 수렴'이지 실패가 아니다. 탈락으로 두면
# 런처가 해석 실패로 분류해 정상 설계점이 통째로 후보에서 빠진다 (§5 ▶수렴·발산 분류).
_OK_STATUS = ('converged', 'limit_cycle')


def converge(m_fixed: float, resp_of, options=None) -> WghtOut:
    """① MTOW 고정점 반복.  ICD0-008 §5 ▶사이징 루프 상세

    m_fixed : MTOW 와 무관한 질량 합 [kg] — 런처가 조립해서 넘긴다
    resp_of : MTOW -> (응답질량_합, payload) 인 **순수 함수**. 같은 MTOW 에 항상
              같은 값이 나와야 하며, 깨지면 배치 실행에서 설계점끼리 오염된다.

    초기값 MTOW₀ = k_init × m_fixed — 수렴값에 영향 없고 반복 횟수에만 영향을 준다.

    질량 특성(x_cg·J)은 여기서 내지 않는다. 위치를 주는 geom.layout 이 ②라
    수렴 뒤에야 계산할 수 있다 → mass_props() 참조. [확정 필요 — ICD §5.1 은
    x_cg·J 를 WGHT(①) 출력으로 적고 있으나 layout 은 ②다]
    """
    res = _iterate(k.k_init * m_fixed, m_fixed, resp_of, options)
    ok = res['status'] in _OK_STATUS
    return WghtOut(
        MTOW=res['MTOW'],
        g4=1.0 if ok else -1.0,
        status=res['status'],
        S_hat=res['S_hat'],
        err=res['err'],
        n_iter=res['n_iter'],
        payload=res['payload'],
        history=res['history'],
    )


def growth_split(history: list, resp_parts_of) -> dict:
    """성장계수 분해 — 각 응답 질량의 민감도 S_i ≈ Δm_i/ΔMTOW (§5 ▶수렴·발산 분류).

    Ŝ_total = Σ S_i 이며 '스노우볼에 누가 기여하는가'의 지도가 된다.

    resp_parts_of : MTOW → {이름: 응답질량 [kg]}. None 이면 분해하지 않는다.
        resp_of 와 **같은 계산**이어야 한다 — 런처가 같은 클로저로 만든다.

    ⚠ 이것은 국소 미분이 아니라 **할선**이다. 구조 응답이 슬라이서 이산화 때문에
      계단형이라(로컬 개정 11-39) 국소 미분은 0 이거나 무한대로 나온다. 유한한
      구간에 걸친 기울기를 봐야 의미가 있다.

    ⚠ resp_parts_of 를 2회 더 호출한다 — 설계점당 비용이 그만큼 늘어난다.
      DOE 배치에서 부담되면 None 을 넘겨 끈다.

    [P7 에서 구현] 수렴 상태머신(_iterate·converge)은 건드리지 않았다.
    """
    if resp_parts_of is None or len(history) < 2:
        return {"struct": None, "motor": None, "batt": None}

    # 이력의 인접 두 점을 쓴다. 다만 수렴 근처에서는 점들이 뭉쳐 있어 차분이 잡음에
    # 묻히므로, 뒤에서부터 충분히 벌어진 짝을 찾는다.
    M_b = history[-1][0]
    M_a = None
    for M, _, _ in reversed(history[:-1]):
        if abs(M - M_b) > k.h_split_min * max(abs(M_b), 1e-12):
            M_a = M
            break
    if M_a is None:
        M_a = M_b * (1.0 - k.h_split)      # 이력이 전부 뭉쳐 있다 — 의도적으로 벌린다

    pa, pb = resp_parts_of(M_a), resp_parts_of(M_b)
    dM = M_b - M_a
    if abs(dM) < 1e-15:
        return {"struct": None, "motor": None, "batt": None}
    return {key: (pb[key] - pa[key]) / dM for key in pa}


def mass_props(MTOW: float, bd: list, l_body: float) -> MassProps:
    """② 배치 확정 후 1회 — 무게중심 · 3축 관성.

    bd : [MassItem] 전체 질량 분해표. 위치는 geom.layout 이 정한 x_parts 에서 온다.
         bd[0] 은 동체 쉘이어야 한다 (아래 길이항 J 가 그것만 쓴다).

    항등식 MTOW == Σbd 를 깨지 않는다 — 질량 항목 분류가 목록을 못 덮는 날,
    조용히 틀린 무게가 나가는 대신 거기서 즉시 멈춘다.
    """
    m_tot = sum(i.m for i in bd)
    if abs(m_tot - MTOW) > 1e-9 * max(MTOW, 1.0):
        raise ValueError(
            f"breakdown 합({m_tot:.9f} kg)이 수렴값({MTOW:.9f} kg)과 다릅니다 "
            f"— 질량 항목 분류 누락 (차이 {(m_tot - MTOW) * 1000:+.3f} g)")

    x_cg = sum(i.m * i.x for i in bd) / m_tot

    # 3축 관성 (x=롤, y=피치, z=요 · 축대칭 → J_yy=J_zz)
    J_xx = sum(i.m * i.r ** 2 for i in bd)
    J_shell_len = bd[0].m * l_body ** 2 / 12.0        # 동체 쉘만 길이항
    J_yy = sum(i.m * ((i.x - x_cg) ** 2 + 0.5 * i.r ** 2) for i in bd) + J_shell_len
    J_zz = J_yy

    return MassProps(x_cg=x_cg, J_xx=J_xx, J_yy=J_yy, J_zz=J_zz, breakdown=bd)
