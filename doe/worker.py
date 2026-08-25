"""워커 — 설계점 하나를 끝까지 평가해 로그 한 줄로 만든다. 자식 프로세스에서 돈다.

**모듈 층에 둔다.** Windows 는 프로세스를 spawn 으로 띄우므로 워커 함수가
`__main__` 에 있으면 자식이 진입 스크립트를 다시 import 한다. 이름으로 찾아갈 수
있는 곳에 두는 편이 안전하다.
"""
from __future__ import annotations

import time

import main
from doe import row


def evaluate_point(p, split: bool = False) -> dict:
    """`main.evaluate` 를 그대로 부른다 — 사이징 경로는 여기서 한 줄도 바뀌지 않는다.

    `split=False` 가 기본이다. Ŝ 분해는 `resp_of` 를 2 회 더 부르므로 설계점당
    +16 % 인데(docs §11-46) 배치에서는 진단용이라 끈다. 수치 경로는 안 바뀐다.
    """
    t0 = time.perf_counter()
    try:
        r = main.evaluate(p.dv, split=split)
        return row.flatten(p, r, time.perf_counter() - t0)
    except Exception as e:              # KeyboardInterrupt 는 삼키지 않는다
        return row.error_row(p, e, time.perf_counter() - t0)
