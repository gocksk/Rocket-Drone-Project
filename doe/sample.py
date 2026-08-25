"""표본 생성 — 상자 + seed → 설계점 목록. **결정론적이고 순수하다.**

같은 (상자, n, seed, n_ser 수준) 은 언제 어디서 돌려도 같은 목록을 낸다.
이게 깨지면 로그를 나중에 재현할 수 없고, 재개(resume)도 다른 점을 이어붙인다.
"""
from __future__ import annotations

import random
from dataclasses import dataclass

from interfaces import DesignVars
from doe import space


@dataclass(frozen=True)
class Point:
    id: int                  # 로그의 기본키. 재개는 이 id 로 건너뛴다
    dv: DesignVars
    dup_of: int | None = None  # 순수성 감시용 복제라면 원본 id


def _lhs(n: int, d: int, rng: random.Random) -> list:
    """라틴 하이퍼큐브 — 축마다 n 층으로 자르고 층당 한 점, 축별로 따로 섞는다.

    `scipy.stats.qmc` 를 쓰지 않는다. 이 저장소는 표준 라이브러리만 쓰고 CI 도
    의존성을 하나도 설치하지 않는다 — 10 줄짜리를 위해 그 계약을 깰 이유가 없다.
    `random.Random(seed)` 는 판·플랫폼에 걸쳐 같은 수열을 낸다 (문자열 시드는
    sha512 로 초기화되므로 이것도 안정적이다).

    균일 격자와 달리 **축마다 n 개 층이 정확히 한 번씩** 쓰인다 — 표본이 어느
    축에서도 뭉치지 않는다. 상자가 얇은 방향(docs §11-48)에서 특히 중요하다.
    """
    cols = []
    for _ in range(d):
        col = [(i + rng.random()) / n for i in range(n)]
        rng.shuffle(col)
        cols.append(col)
    return [[cols[j][i] for j in range(d)] for i in range(n)]


def build(box: dict, n: int, seed: int, n_ser_levels=(6,),
          dup_frac: float = 0.01) -> list:
    """설계점 목록. `n` 은 **셀 수 수준 하나당** 표본 수다.

    `n_ser` 는 이산이라 축에 넣지 않고 수준별로 배치를 나눈다 (ICD §2.1).
    수준마다 시드를 갈라 서로 다른 표본을 뽑는다 — 같은 표본을 셀 수만 바꿔
    돌리면 비교는 깔끔하지만 형상 공간을 그만큼 덜 훑는다. [확정 필요]

    `dup_frac` — 표본의 이 비율만큼을 **같은 설계점으로 한 번 더** 넣는다.
    병렬 실행에서 설계점끼리 오염되면 두 행의 결과가 갈라지므로 그때 잡힌다
    (CLAUDE.md 규칙 3 의 배치판 검사). 비용은 그 비율 그대로다.
    """
    pts, pid = [], 0
    for level in n_ser_levels:
        rng = random.Random(f"{seed}:{level}")
        for u in _lhs(n, len(space.AXES), rng):
            vals = {a: box[a][0] + u[j] * (box[a][1] - box[a][0])
                    for j, a in enumerate(space.AXES)}
            pts.append(Point(id=pid, dv=space.make_dv(vals, level)))
            pid += 1

    if dup_frac > 0.0:
        step = max(1, round(1.0 / dup_frac))
        for p in pts[::step]:
            pts.append(Point(id=pid, dv=p.dv, dup_of=p.id))
            pid += 1
    return pts
