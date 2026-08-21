# 통합 사이징 코드 스켈레톤

ICD0-007과 모듈 가이드라인 11개를 코드 구조로 옮긴 것.
**지금 상태로 실행됩니다** — 물리 자리에 가짜 숫자(스텁)가 들어 있을 뿐입니다.

```
python main.py        # 전체 파이프라인 실행 (수렴 + 합격 판정 + 성적표)
python common/atm.py  # ATM 단위 검산
```

## 폴더 구조 = 담당 구조

| 파일 | 담당 | 상태 |
|---|---|---|
| `main.py` `interfaces.py` | **통합** | 실제 구현 — 다른 사람은 수정 금지 |
| `constants.py` | 통합 (값은 각 담당이 제안) | ★ = 잠정값, 「할 일 통합 목록」 참조 |
| `common/atm.py` | 통합 | 실제 구현 (검산 포함) |
| `common/out.py` | 통합 | 실제 구현 (출력 인코딩 고정 — 진입점에서만 호출) |
| `common/srl.py` | 조사 담당 | **[스텁]** → 스펙표 회귀로 교체 |
| `modules/geom.py` | GEOM | 가이드라인 수식 구현됨 → **검토·보완** |
| `modules/aero.py` | AERO | 골격 구현, `cd_2d`만 스텁 |
| `modules/prop_a.py` | PROP | **[스텁]** → BEMT 테이블+이분법으로 교체 |
| `modules/strc.py` | STRC | 골격 구현, `k_sl_*` 계수만 실측 필요 |
| `modules/wght.py` | 통합 | 실제 구현 (수렴 루프) |
| `modules/prop_b.py` | PROP | 구조 구현 (PROP-A 교체되면 자동으로 진짜가 됨) |
| `modules/miss.py` | MISS | 구조 구현 (시간전진) |
| `modules/stab.py` | STAB | 구조 구현 |
| `modules/cost.py` | COST | 골격 구현 (단가는 SRL 의존) |

## 작업 규칙 세 가지

1. **자기 파일만 수정한다.** `# [스텁]` 표시된 줄을 가이드라인 수식으로 바꾸는 것이
   각자의 일이다. 함수의 **입출력(서명)은 절대 바꾸지 않는다.**
2. **입출력을 바꾸고 싶으면 코드가 아니라 ICD부터.** `interfaces.py`가 ICD §5의
   코드판이다. 노션 ICD 변경 절차(§9) → 통합 담당이 `interfaces.py` 반영 → 그 다음 구현.
3. **숫자를 코드에 직접 박지 않는다.** 상수는 전부 `constants.py`에서 가져온다.
   새 상수가 필요하면 거기 추가하고 노션 가이드라인 「확정해야 할 상수」에도 등재.

## GitHub 작업 흐름

위 「작업 규칙」이 무엇을 고치나라면, 여기는 어떻게 올리나다.

### 1. 브랜치 파기

`main`에 직접 푸시 금지. 항상 최신 main에서 딴다.

```bash
git switch main && git pull
git switch -c feat/geom
```

이름 규칙 — `feat/<모듈>` (스텁 교체·기능) · `fix/<내용>` (수정) · `docs/<내용>` (문서).

### 2. PR 전 확인

```bash
python main.py
```

**에러 없이 끝나기만 하면 된다.** 합격/불합격(g6 FAIL 등)은 상관없음 —
중간에 죽지만 않으면 통과.

### 3. 커밋

제목에 모듈명을 단다 (예: `[AERO] cd_2d를 NeuralFoil 테이블로 교체`).

- 성격이 다른 변경은 **커밋을 나눈다.** 나중에 한쪽만 되돌릴 수 있다.
- 미결 사항은 커밋 메시지에 `[확정 필요]`로 남긴다.

### 4. push → PR

```bash
git push -u origin feat/geom
```

push하면 터미널에 PR 생성 링크가 뜬다. 본문에는 **무엇을 검증했는지**와
**미결 사항**을 적는다. PR은 Github 웹에서 진행하면 된다.

### 5. 머지

**Create a merge commit** 또는 **Rebase and merge**를 쓴다.

⚠️ **Squash and merge는 쓰지 않는다** — 커밋이 하나로 합쳐져 일부만 되돌릴 수 없고,
메시지에 남긴 `[확정 필요]`가 묻힌다.

### 6. 머지 후 정리

GitHub의 **Delete branch** 버튼을 누르고, 로컬도 정리한다.

```bash
git switch main && git pull
git branch -d feat/geom
```

`-d`는 머지된 브랜치만 지우므로 실수 방지가 된다.

### 남이 머지했을 때

작업 중인 브랜치에 최신 main을 반영한다.

```bash
git pull origin main --rebase
```

`constants.py`는 전원이 건드리는 파일이라 **충돌이 가장 잘 난다.**
브랜치 시작 전 pull, 머지 소식 들으면 rebase — 이 둘만 지키면 대부분 예방된다.

## 참고 문서 (노션)

- ICD0-007 — 통합 사이징 코드 인터페이스 문서 (변수명·단위의 원본)
- 각 모듈 「계산 가이드라인」 — 교체할 수식의 원본
- 「할 일 통합 목록 — C-1 코드 착수 전」 — ★ 상수 확정 현황
