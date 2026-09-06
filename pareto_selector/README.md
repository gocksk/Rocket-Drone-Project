# Feasible-only Pareto 선택기

`feasible=True`인 행만 사용해 Pareto front를 계산하고, 그 안에서 가중 정규화 점수가 가장 낮은 대표 설계안을 선택합니다. 데이터 행 수는 달라도 됩니다.

## 현재 데이터용 기본 목적함수

| 열 | 방향 | 의미 |
|---|---|---|
| `ec.C1_MTOW[kg]` | 최소화 | 이륙중량 |
| `ec.C2_R_dash[m]` | 최대화 | 항속거리 |
| `ec.C3_margin_V` | 최대화 | 전압 여유 |
| `ec.C5_alpha_max[rad/s2]` | 최대화 | 기동성 |
| `ec.C6_SPL_hover[dB]` | 최소화 | 호버 소음 |
| `ec.C7_Cost_acq[KRW]` | 최소화 | 획득 비용 |

목적함수의 방향이나 열이 달라지면 `--objectives` 또는 코드 상단의 `DEFAULT_OBJECTIVES`를 수정하세요. `g.g8`, `g.g9`는 이미 `feasible` 열에 반영되어 있으므로 별도로 제약식에 다시 넣지 않습니다.

## 저장소 루트에서 실행

저장소 루트에서 DOE 로그를 CSV로 변환한 뒤 선택기를 실행합니다.

```bash
python -m doe.export runs/smoke.jsonl --out runs/smoke.csv
python -m pareto_selector runs/smoke.csv --output runs/smoke_pareto
```

필요한 패키지는 저장소 환경에 설치합니다.

```bash
pip install -r pareto_selector/requirements.txt
```

Windows PowerShell에서는 `python` 대신 `py`를 사용해도 됩니다.

실행 뒤 `runs/smoke_pareto` 폴더에 다음 파일이 생성됩니다.

- `pareto_front.csv`: 서로 지배되지 않는 모든 feasible 설계안
- `selected_design.csv`: 가중 정규화 점수 기준의 최종 추천 설계안 1개

## 목적함수·가중치 변경 예시

질량(작을수록 좋음)과 항속거리(클수록 좋음)만 60:40으로 평가하려면:

```bash
python -m pareto_selector runs/smoke.csv \
  --objectives "ec.C1_MTOW[kg]:min,ec.C2_R_dash[m]:max" \
  --weights "0.6,0.4" \
  --output runs/smoke_pareto_two_objectives
```

가중치는 **Pareto front를 만드는 과정에는 쓰이지 않고**, Pareto 해들 중 대표안 하나를 고를 때만 쓰입니다. 따라서 가중치를 바꿔도 `pareto_front.csv`는 같고 `selected_design.csv`만 달라질 수 있습니다.

CSV 열 이름이 `ec.C1_MTOW[kg]`처럼 점을 포함하는 원본 형식이거나 `ec_C1_MTOW[kg]`처럼 언더스코어로 변환된 형식이어도 인식합니다. `feasible` 열이 없으면 모든 행을 분석하고, `id` 열이 없으면 `source_id`를 결과 식별자로 사용합니다.
