"""응답 질량 스텁 — WGHT 수렴 루프 검증용 가짜 구조 모듈

실제 modules/strc.py 는 슬라이서 계수(k_sl_*)가 실측 대기라 '정답'을 모른다.
수렴 상태머신이 맞게 도는지 보려면 정답을 아는 구조 모델이 필요하다. 그게 이 파일이다.

[ICD0-008 계약 — 반환 형태만 바뀌었다]
주입 함수가 StrcOut 하나가 아니라 (응답질량_합, payload) 를 돌려준다.
이 스텁들은 응답 질량 중 구조 항만 모형화하므로 합 == W_str 이고, payload 로는
StrcOut 을 그대로 실어 보낸다 — 그래야 '반환값이 어느 분기의 것인가'를 대조할 수 있다.
수학·판정 논리는 하나도 바뀌지 않았다.

[왜 선형 스텁이 기본인가]
멱함수 W_str = a·MTOW^b 는 b != 1 일 때 구조 비중이 반복 중에 변한다. 그러면 Ŝ 가
상수가 아니라서 '경계가 Ŝ=1 이다'라는 이론값과 대조할 수 없다.
선형 W_str = S·MTOW 는 S 가 기울기 그 자체이고 고정점이 해석적으로 나온다:

    MTOW* = m_fixed / (1 - S)

비선형성의 영향은 멱함수 스텁으로 따로 본다.

[모든 스텁은 순수하다]
파라미터를 클로저로 잡을 뿐 상태를 갱신하지 않는다. _iterate() 의 무상태성은
주입 함수가 무상태일 때만 성립하기 때문이다.
그 원칙을 어기면 어떻게 되는지 보이려고 make_impure_strc 만 예외로 둔다 — 테스트 전용이다.
"""
import math

from interfaces import StrcOut, MassItem


def _out(W_str, l_body=1.0, x_cg_str=0.5, r_body=0.05):
    """스텁의 (응답질량_합, payload). 구조를 길이 l_body 인 균일 봉으로 근사한다.

    실제 STRC 가 오면 통째로 대체된다 — 여기서 중요한 건 값의 정확도가 아니라
    W_str 이 MTOW 에 어떻게 반응하는가다. breakdown_str[0] 은 wght.mass_props 가
    동체 쉘로 쓰므로(길이항 J) 최소 한 항목은 있어야 한다.
    """
    bd = [MassItem("shell", W_str, x_cg_str, r_body)]
    return W_str, StrcOut(W_str=W_str, m_print=W_str * 1.05, g5=1.0, breakdown_str=bd)


def make_linear_strc(S, **kw):
    """W_str = S · MTOW.  고정점 MTOW* = m_fixed/(1-S) 가 해석적으로 나온다.

    S >= 1 이면 고정점이 음수 = 애초에 존재하지 않는다. 그래서 발산은 수치 문제가
    아니라 구조 문제이고, 경계는 beta 와 무관하게 정확히 S=1 이다.
    """
    def resp_of(MTOW):
        return _out(S * MTOW, **kw)
    return resp_of


def make_power_strc(a, b, **kw):
    """W_str = a · MTOW^b.  비선형성 확인용.

    국소 민감도 S(MTOW) = a·b·MTOW^(b-1) 이 MTOW 에 의존한다. 즉 반복 중에 S 가
    변하므로, 수렴 후 Ŝ 는 '수렴점 근방의 국소 미분계수'다.
    """
    def resp_of(MTOW):
        return _out(a * MTOW ** b, **kw)
    return resp_of


def make_quantized_strc(S, quantum, **kw):
    """W_str = quantum · round(S·MTOW / quantum).  계단형 출력.

    실제 STRC 는 슬라이서 회귀라 레이어 수·인필·둘레 수가 정수다. 그래서 출력이
    계단형일 수 있고, 그러면 MTOW 가 두 값 사이를 무한 진동한다:

        r̂ -> -1,  |resid| 는 양자 크기에서 멈춤,  무한 지속

    이것을 발산으로 분류하면 특정 형상대의 설계점이 통째로, 조용히 후보에서 빠진다.
    limit_cycle 을 별도 status 로 둔 이유다.
    """
    if quantum <= 0:
        raise ValueError("quantum 은 양수여야 합니다.")

    def resp_of(MTOW):
        return _out(quantum * round(S * MTOW / quantum), **kw)
    return resp_of


def make_noisy_strc(base_fn, rel_noise, seed=0):
    """base_fn 출력에 재현 가능한 '수치 노이즈'를 얹는다 — resid_floor 실측 연습용.

    난수를 쓰지 않는다. MTOW 값 자체를 흔들어 결정론적으로 만든다. 난수를 쓰면 같은
    MTOW 에 다른 값이 나와 '순수 함수' 규약을 스텁이 스스로 어기고, 재현이 안 돼
    실측 도구로 쓸 수 없다.
    """
    def resp_of(MTOW):
        m, _ = base_fn(MTOW)
        h = math.sin((MTOW * 1e6 + seed) % 6.283185307179586)
        return _out(m * (1.0 + rel_noise * h))
    return resp_of


def make_impure_strc(S):
    """상태를 들고 있는 '나쁜' 스텁 — 무상태성 대조가 이걸 잡아야 한다.

    실제로 이런 코드가 생기는 경로는 warm start 캐시다. 테스트 전용이며 다른 곳에서
    쓰지 말 것.
    """
    state = {'n': 0}

    def resp_of(MTOW):
        state['n'] += 1
        return _out(S * MTOW * (1.0 + 1e-3 * state['n']))   # 호출 횟수에 따라 답이 변한다
    return resp_of
