"""`Result` → 로그 한 줄. **`Result` 를 아는 유일한 곳이다.**

워커가 여기서 평면화해야 하는 이유가 있다: `Result.pre.aero.F_drag` 는 클로저라
pickle 이 안 된다. `Result` 를 그대로 부모 프로세스로 돌려보낼 수 없다.

로그는 JSONL 이다. 진단 항목이 아직 계속 늘어나는 중이라(§11 개정 진행 중)
열 고정 형식(CSV)을 쓰면 과거 로그와 안 붙는다.
"""
from __future__ import annotations

import dataclasses
import json
import traceback

from interfaces import Result

# 순수성 비교에서 뺄 항목 — 실행마다 달라도 되는 것들.
_SIG_EXCLUDE = {"wall_s", "exc"}


def _safe(v):
    """JSON 으로 나갈 수 있는 형태로. 모르는 것은 버리지 말고 repr 로 남긴다."""
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, (list, tuple)):
        return [_safe(x) for x in v]
    if isinstance(v, dict):
        return {str(a): _safe(b) for a, b in v.items()}
    return repr(v)


def flatten(p, r: Result, wall: float) -> dict:
    """평가 성공 행. g·EC·진단을 통째로 싣는다 — 나중에 뭘 볼지 지금 모른다."""
    return {
        "id": p.id,
        "dup_of": p.dup_of,
        "dv": dataclasses.asdict(p.dv),
        "feasible": bool(r.feasible),
        "fail_code": r.fail_code,
        "fail_stage": r.fail_stage,
        "g": {n: _safe(v) for n, v in r.g.items()},
        "ec": {n: _safe(v) for n, v in r.ec.items()},
        "diag": {n: _safe(v) for n, v in r.diag.items()},
        "wall_s": wall,
    }


def error_row(p, exc: BaseException, wall: float) -> dict:
    """예외로 죽은 행. **한 설계점이 배치를 죽이면 안 된다.**

    `GeomInfeasible` 은 이미 `main.evaluate` 안에서 사유 코드로 처리된다. 여기
    걸리는 것은 그 밖의 것 — 트림 실패나 아직 모르는 것들이다. 사유 코드를
    `exception:<타입>` 으로 남겨 두면 DOE 보고에서 물리 탈락과 섞이지 않는다.
    """
    return {
        "id": p.id,
        "dup_of": p.dup_of,
        "dv": dataclasses.asdict(p.dv),
        "feasible": False,
        "fail_code": f"exception:{type(exc).__name__}",
        "fail_stage": str(exc).splitlines()[0][:200] if str(exc) else "",
        "g": {}, "ec": {}, "diag": {},
        "exc": "".join(traceback.format_exception(exc))[-1200:],
        "wall_s": wall,
    }


def signature(row: dict) -> str:
    """순수성 비교용 지문. 같은 설계점이면 **글자까지 같아야** 한다.

    파이썬의 float→JSON 은 repr 이라 왕복이 정확하다 — 즉 이 문자열 비교는
    비트 비교와 같다.
    """
    core = {a: b for a, b in row.items()
            if a not in _SIG_EXCLUDE and a not in ("id", "dup_of")}
    return json.dumps(core, sort_keys=True, ensure_ascii=False)
