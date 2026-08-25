"""DOE 탐색 공간 — 표본을 뿌릴 상자와 그 조절 규칙.

[중요] 여기 숫자는 **물리 상수가 아니다.** 배치마다 바뀌는 실행 설정이라
`constants.py` 가 아니라 여기 둔다. 대신 실행할 때마다 상자 전체가 매니페스트에
찍히므로 "이 로그가 어느 상자에서 나왔나" 는 로그 파일만 보면 안다.
CLAUDE.md 규칙 2(숫자를 코드에 박지 않는다)와 부딪히는 지점이라 명시해 둔다.
[확정 필요 — constants.py 로 옮길지]

**표본 공간의 축은 설계변수와 하나 다르다** — `x_fin` 대신 `x_fin_ratio = x_fin/l_body`.
절대 위치로 뿌리면 전장(`l_body = d_body·lambda_body`)과 무관하게 뽑혀서
"전장 0.6 m 기체에 핀을 0.8 m 지점에" 같은 조합이 대량으로 나오고, 그 표본은
⓪ GEOM 에서 통째로 죽는다 (`geom.hull` 의 형상 성립 조건 세 개).

상자는 **합격 영역이 아니다.** 표본을 뿌릴 범위일 뿐이다. 너무 넓으면 표본이
탈락으로 낭비되고(로컬 개정 11-48: `d_body`·`x_fin`·`k_mot` 은 ±4 % 에서 9 점 중
4 점을 잃는다), 너무 좁으면 최적점이 상자 벽에 붙어 결론이 안 된다.
"""
from __future__ import annotations

import functools
import json

import constants as k
from interfaces import DesignVars
from modules import atm, prop
# 공칭 설계점의 정의는 P7 게이트에 있다. 여기서 다시 적으면 조용히 어긋난다.
from validation.p7_gate import DV0 as NOMINAL

# 표본 공간의 축 — **순서가 고정이다.** 이 순서가 바뀌면 같은 seed 가 다른 표본을
# 낸다. 축을 추가·제거하면 과거 로그와 재현성이 끊긴다.
AXES = ("d_body", "lambda_body", "S_fin", "x_fin_ratio", "AR_fin", "f_mount",
        "n_design", "d_prop", "pd_prop", "k_E", "k_mot")

# `n_ser` 는 이산이라 축이 아니다 — 수준별로 배치를 나눠 돌린다 (ICD §2.1).
N_SER_LEVELS = (4, 6, 8)

SMOKE_SPAN = 0.04         # 스모크 상자 폭 — P7 연속성 스윕과 같은 ±4 % (docs §11-47)


def l_body_of(d_body: float, lambda_body: float) -> float:
    """전장 [m]. ⚠ 이 식의 원본은 `geom.hull` 이다 — 여기가 두 번째 집이다.

    `x_fin_ratio` 를 절대 위치로 바꾸려면 `DesignVars` 를 만들기 **전에** 전장이
    필요한데, `geom.hull(dv)` 는 그 `DesignVars` 를 인자로 받으므로 못 부른다.
    한쪽만 고치면 표본이 조용히 어긋난다. `geom.l_body_of(...)` 접근자를 두고
    양쪽이 그걸 부르게 하는 편이 맞다고 본다 [확정 필요].
    """
    return d_body * lambda_body


@functools.lru_cache(maxsize=1)
def pd_prop_lo() -> float:
    """`pd_prop` 의 물리적 하한 — ICD §2 하한 규칙. 이건 설정이 아니라 계산값이다.

    설계점과 무관(순항속도와 음속만 쓴다)하므로 한 번만 구한다.
    """
    return prop.pd_prop_min(k.V_cr, atm.run(k.h_miss).a_snd)


def nominal_sample() -> dict:
    """공칭 설계점을 **표본 공간 좌표**로. `--focus` 의 중심이자 스모크 상자의 중심."""
    v = {a: getattr(NOMINAL, a) for a in AXES if a != "x_fin_ratio"}
    v["x_fin_ratio"] = NOMINAL.x_fin / l_body_of(NOMINAL.d_body, NOMINAL.lambda_body)
    return v


def _pm(x: float, span: float, lo: float | None = None,
        hi: float | None = None) -> tuple:
    a, b = x * (1.0 - span), x * (1.0 + span)
    return (a if lo is None else max(a, lo)), (b if hi is None else min(b, hi))


def box_smoke() -> dict:
    """배관 확인용 — 공칭점 ±4 %. **스크리닝 상자가 아니다.**

    P7 이 이 폭에서 이미 재 봤으므로 대부분 살아남는다는 것을 안다. 여유 변수만
    하한을 1.0 으로 자른다 — `k_mot < 1.0` 은 전량 열로 탈락하고(docs §11-44),
    `k_E < 1.0` 은 ICD §2.2 범위 밖이다.
    """
    v = nominal_sample()
    b = {a: _pm(v[a], SMOKE_SPAN) for a in AXES}
    # ICD §2 가 준 경계는 ±4 % 라도 넘지 않는다 — 넘으면 그 표본은 규칙 밖이라
    # 배관 확인이 아니라 "규칙 밖에서 뭐가 나오나" 를 보는 실행이 돼 버린다.
    b["f_mount"] = _pm(v["f_mount"], SMOKE_SPAN, hi=1.0)          # ICD §2.1 상한
    b["n_design"] = _pm(v["n_design"], SMOKE_SPAN, lo=4.0)        # ICD §2.1 하한
    b["pd_prop"] = _pm(v["pd_prop"], SMOKE_SPAN, lo=pd_prop_lo())  # §2 하한 규칙
    b["k_E"] = _pm(v["k_E"], SMOKE_SPAN, lo=1.0)                  # ICD §2.2 하한
    b["k_mot"] = _pm(v["k_mot"], SMOKE_SPAN, lo=1.0)              # docs §11-44
    return b


def box_screen() -> dict:
    """1차 스크리닝 상자 — **다섯 축이 비어 있다.**

    ICD §2 가 범위를 준 축만 채웠다. 나머지는 §8 B-2(미확정)이고, 스크리닝의
    목적 자체가 그걸 정하는 것이라 순환이다. 그럴듯한 숫자로 채우면 확정된 것으로
    착각하게 되므로 `None` 으로 두고, 그 상태로 실행하면 런처가 멈춘다.
    `--set` 으로 넣어야 돈다.
    """
    return {
        "d_body": None,          # ICD §2.1 TBD
        "lambda_body": None,     # ICD §2.1 TBD
        "S_fin": None,           # ICD §2.1 TBD
        "x_fin_ratio": None,     # ICD §2.1 TBD (x_fin 이 TBD)
        "AR_fin": None,          # ICD §2.1 TBD
        "f_mount": (0.4, 1.0),   # ICD §2.1
        "n_design": (4.0, 10.0),  # ICD §2.1 — EC C4 이기도 하다
        "d_prop": (0.10, 0.18),  # ICD §2.1 (TBD 표시 붙어 있음)
        "pd_prop": None,         # 하한만 규칙으로 나온다 (pd_prop_lo) · 상한 TBD
        "k_E": (1.0, 1.5),       # ICD §2.2
        "k_mot": (1.0, 1.4),     # ICD §2.2 · 하한 1.0 은 docs §11-44 가 확인
    }


def box_main() -> dict:
    """본 DOE 상자 — **전 축이 비어 있다.** 스크리닝이 만들어 주는 것이기 때문이다.

    여기에 숫자를 미리 적어 두면 안 된다. 본 DOE 의 범위는 스크리닝에서 유효율과
    탈락 축을 보고 좁히거나 넓힌 결과이고, 그 전에 적은 값은 근거가 없는데도
    "확정된 범위" 로 굳는다.

    쓰는 법은 둘이다.
      · `--box-file runs/screen.manifest.json` — 스크리닝 실행의 상자를 그대로 받는다
      · `--box-file box_main.json` + `--set` — 받아서 손보고 쓴다

    이름을 따로 두는 이유는 매니페스트의 `box_name` 이다. 몇 달 뒤 로그 더미에서
    "이건 스크리닝이었나 본 DOE 였나" 를 파일만 보고 알 수 있어야 한다.
    """
    return {a: None for a in AXES}


BOXES = {"smoke": box_smoke, "screen": box_screen, "main": box_main}


def load_box(path: str) -> dict:
    """상자를 파일에서 읽는다. **상자 JSON 과 실행 매니페스트를 둘 다 받는다.**

    스크리닝을 돌리면 상자가 이미 매니페스트에 통째로 찍혀 있다. 그걸 그대로
    다음 단계의 입력으로 쓸 수 있어야 손으로 --set 을 열한 번 다시 치지 않는다
    (그 과정에서 한 축을 잘못 옮겨 적으면 다음 배치 전체가 조용히 어긋난다).
    """
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    if isinstance(d.get("box"), dict):        # 매니페스트다 — 상자만 꺼낸다
        d = d["box"]
    box = {}
    for a in AXES:
        v = d.get(a)
        box[a] = None if v is None else (float(v[0]), float(v[1]))
    return box


def save_box(box: dict, path: str) -> None:
    """상자를 JSON 으로. 매니페스트와 같은 형식이라 서로 오간다."""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(describe(box), f, ensure_ascii=False, indent=1)


def parse_set(s: str) -> tuple:
    """`d_body=0.07:0.11` → ("d_body", (0.07, 0.11))"""
    name, _, rng = s.partition("=")
    lo, _, hi = rng.partition(":")
    if name not in AXES or not hi:
        raise ValueError(f"--set 형식이 아니다: {s!r}  (예: d_body=0.07:0.11)")
    return name, (float(lo), float(hi))


def parse_focus(s: str) -> tuple:
    """`d_body=0.5` → ("d_body", 0.5). 공칭점을 중심으로 폭을 배율만큼 줄인다.

    §11-48 의 칼날축(`d_body`·`x_fin`·`k_mot`)을 촘촘히 보는 손잡이다. LHS 는 모든
    축을 같은 층수로 자르므로 "축마다 표본을 더 준다" 는 게 성립하지 않는다 —
    촘촘하게 본다는 것은 결국 **그 축의 상자를 좁힌다**는 뜻이고, 그래서 별도의
    가중 분포를 지어내지 않고 폭으로 표현한다.
    """
    name, _, f = s.partition("=")
    if name not in AXES or not f:
        raise ValueError(f"--focus 형식이 아니다: {s!r}  (예: d_body=0.5)")
    return name, float(f)


def resolve(name: str, sets=(), focuses=(), file: str | None = None) -> dict:
    """상자 이름 + 파일 + CLI 조절 → 최종 상자.

    순서는 **이름 → 파일 → `--set` → `--focus`** 다. 뒤엣것이 앞엣것을 덮는다.
    """
    box = dict(BOXES[name]())
    if file:
        box.update({a: v for a, v in load_box(file).items() if v is not None})
    for n, rng in sets:
        box[n] = rng
    nom = nominal_sample()
    for n, f in focuses:
        if box[n] is None:
            raise ValueError(f"--focus {n}: 상자가 비어 있다. --set 으로 먼저 채운다")
        lo, hi = box[n]
        c = nom[n]
        box[n] = (c - (c - lo) * f, c + (hi - c) * f)
    return box


def missing(box: dict) -> list:
    return [a for a in AXES if box.get(a) is None]


def validate(box: dict) -> list:
    """치명적이지 않은 경고 목록. 상자가 이상해도 멈추지는 않고 로그에 남긴다."""
    w = []
    for a in AXES:
        lo, hi = box[a]
        if lo >= hi:
            raise ValueError(f"상자 {a} 의 하한이 상한 이상이다: {lo} ≥ {hi}")
    if box["pd_prop"][0] < pd_prop_lo():
        w.append(f"pd_prop 하한 {box['pd_prop'][0]:.4f} 가 §2 하한 규칙 "
                 f"{pd_prop_lo():.4f} 미만 — 그 아래 표본은 g1 에서 떨어진다")
    if box["x_fin_ratio"][1] >= 1.0:
        w.append("x_fin_ratio 상한이 1.0 이상 — 핀 앞전이 전장 밖이라 ⓪ 에서 죽는다")
    if box["n_design"][0] < 4.0 or box["n_design"][1] > 10.0:
        w.append("n_design 이 ICD §2.1 범위(4–10) 밖까지 간다")
    return w


def make_dv(vals: dict, n_ser: int) -> DesignVars:
    """표본 공간 좌표 → `DesignVars`. 여기서 비율이 절대 위치가 된다."""
    l_body = l_body_of(vals["d_body"], vals["lambda_body"])
    return DesignVars(
        d_body=vals["d_body"], lambda_body=vals["lambda_body"],
        S_fin=vals["S_fin"], x_fin=vals["x_fin_ratio"] * l_body,
        AR_fin=vals["AR_fin"], f_mount=vals["f_mount"],
        n_design=vals["n_design"], d_prop=vals["d_prop"],
        pd_prop=vals["pd_prop"], n_ser=int(n_ser),
        k_E=vals["k_E"], k_mot=vals["k_mot"])


def describe(box: dict) -> dict:
    """매니페스트에 찍을 형태. 상자는 로그와 함께 남아야 의미가 있다."""
    return {a: (list(box[a]) if box[a] is not None else None) for a in AXES}
