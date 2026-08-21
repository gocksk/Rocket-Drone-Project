"""OUT — 출력 스트림 규약 공용 함수. [실제 구현]

콘솔로 나가는 글자는 여기를 거쳐 UTF-8로 통일한다 (ICD §7의 하드코딩 금지와 같은 취지 —
환경마다 다른 기본 인코딩을 각 파일이 따로 처리하지 않는다).
실행 진입점(main.py·common/atm.py)에서만 부르면 된다.
"""
import sys


def stdout_utf8():
    """stdout·stderr을 UTF-8로 고정한다.

    Windows에서 출력이 파이프·파일로 리다이렉트되면 인코딩이 cp949가 되는데,
    cp949에는 U+2014(—)가 없어 성적표를 찍다가 UnicodeEncodeError로 죽는다.
    (콘솔 직접 실행은 PEP 528로 이미 UTF-8이라 무사 — 즉 CI·리다이렉트에서만 터진다.)
    README의 PR 조건 「python main.py가 에러 없이 끝날 것」을 환경과 무관하게 만든다.

    이미 UTF-8이면 아무것도 하지 않는다. 여러 번 불러도 안전하다.
    """
    for s in (sys.stdout, sys.stderr):
        enc = (getattr(s, "encoding", "") or "").lower().replace("-", "")
        if enc != "utf8" and hasattr(s, "reconfigure"):
            s.reconfigure(encoding="utf-8")
