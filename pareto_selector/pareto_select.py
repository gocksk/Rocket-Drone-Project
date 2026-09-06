"""Feasible 데이터에서 Pareto front와 대표 설계안을 선택한다.

예시:
  python pareto_select.py ../upload/smoke.csv --output results
  python pareto_select.py data.csv --objectives "ec.C1_MTOW[kg]:min,ec.C2_R_dash[m]:max" --weights "0.6,0.4"
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

DEFAULT_OBJECTIVES = [
    ("ec.C1_MTOW[kg]", "min"),        # MTOW: 작을수록 좋음
    ("ec.C2_R_dash[m]", "max"),       # 항속거리: 클수록 좋음
    ("ec.C3_margin_V", "max"),        # 전압 여유: 클수록 좋음
    ("ec.C5_alpha_max[rad/s2]", "max"), # 기동성: 클수록 좋음
    ("ec.C6_SPL_hover[dB]", "min"),   # 소음: 작을수록 좋음
    ("ec.C7_Cost_acq[KRW]", "min"),   # 비용: 작을수록 좋음
]

def parse_objectives(text: str | None) -> list[tuple[str, str]]:
    """'열이름:min,열이름:max' 형식의 실행 인자를 읽는다."""
    if not text:
        return DEFAULT_OBJECTIVES
    result = []
    for item in text.split(","):
        if ":" not in item:
            raise ValueError(f"목적함수 형식 오류: {item!r} (예: MTOW:min)")
        name, direction = (part.strip() for part in item.rsplit(":", 1))
        direction = direction.lower()
        if direction not in {"min", "max"}:
            raise ValueError(f"방향은 min 또는 max여야 합니다: {item!r}")
        result.append((name, direction))
    return result


def feasible_mask(series: pd.Series) -> pd.Series:
    """True/true/1/yes 등 흔한 feasibility 표기를 처리한다."""
    return series.astype(str).str.strip().str.lower().isin({"true", "1", "yes", "y", "feasible"})


def resolve_column(columns: pd.Index, requested: str) -> str | None:
    """원본 점 표기와 CSV에서 언더스코어로 변환된 표기를 연결한다."""
    candidates = (requested, requested.replace(".", "_"))
    return next((candidate for candidate in candidates if candidate in columns), None)


def pareto_mask(minimization_values: np.ndarray) -> np.ndarray:
    """모든 목적함수가 최소화 형태인 배열에서 비지배해 여부를 반환한다."""
    n = len(minimization_values)
    is_pareto = np.ones(n, dtype=bool)
    for i in range(n):
        # 어떤 j가 i보다 모든 목적함수에서 같거나 좋고, 하나 이상 엄격히 좋으면 i는 지배됨.
        dominates_i = np.all(minimization_values <= minimization_values[i], axis=1) & np.any(
            minimization_values < minimization_values[i], axis=1
        )
        if np.any(dominates_i):
            is_pareto[i] = False
    return is_pareto


def normalize_for_choice(values: np.ndarray, directions: list[str]) -> np.ndarray:
    """0이 가장 좋고 1이 가장 나쁜 점수가 되도록 min-max 정규화한다."""
    scores = np.zeros_like(values, dtype=float)
    for j, direction in enumerate(directions):
        col = values[:, j]
        low, high = col.min(), col.max()
        if np.isclose(low, high):
            continue
        scores[:, j] = (col - low) / (high - low)
        if direction == "max":
            scores[:, j] = 1.0 - scores[:, j]
    return scores


def main() -> None:
    parser = argparse.ArgumentParser(description="feasible 데이터의 Pareto front 선택기")
    parser.add_argument("csv", type=Path, help="입력 CSV 파일 경로")
    parser.add_argument("--feasible-column", default="feasible", help="feasibility 열 이름 (기본: feasible)")
    parser.add_argument(
        "--objectives",
        help="목적함수. 예: '질량:min,항속거리:max'. 생략 시 이 프로젝트 기본 ec.C 열 사용",
    )
    parser.add_argument("--weights", help="목적함수 순서와 같은 가중치. 예: '0.4,0.3,0.3'")
    parser.add_argument("--id-column", default="id", help="결과를 식별할 열 이름 (기본: id)")
    parser.add_argument("--output", type=Path, default=Path("pareto_results"), help="결과 폴더")
    args = parser.parse_args()

    objectives = parse_objectives(args.objectives)
    df = pd.read_csv(args.csv, encoding="utf-8-sig")
    objective_columns = [resolve_column(df.columns, name) for name, _ in objectives]
    missing = [name for (name, _), column in zip(objectives, objective_columns) if column is None]
    if missing:
        raise SystemExit(f"CSV에 없는 목적함수 열: {missing}\n사용 가능한 열: {', '.join(df.columns)}")
    columns = [column for column in objective_columns if column is not None]

    feasible_column = resolve_column(df.columns, args.feasible_column)
    if feasible_column is None:
        feasible = df.copy()
        print(f"주의: feasibility 열 {args.feasible_column!r}이 없어 전체 {len(df)}개 행을 사용합니다.")
    else:
        feasible = df.loc[feasible_mask(df[feasible_column])].copy()

    for col in columns:
        feasible[col] = pd.to_numeric(feasible[col], errors="coerce")
    complete = feasible.dropna(subset=columns).copy()
    if complete.empty:
        raise SystemExit("목적함수가 모두 수치인 feasible 행이 없습니다.")

    raw_values = complete[columns].to_numpy(dtype=float)
    directions = [direction for _, direction in objectives]
    minimization_values = raw_values.copy()
    for j, direction in enumerate(directions):
        if direction == "max":
            minimization_values[:, j] *= -1

    front = complete.loc[pareto_mask(minimization_values)].copy()
    front_values = front[columns].to_numpy(dtype=float)
    if args.weights:
        weights = np.array([float(x.strip()) for x in args.weights.split(",")])
        if len(weights) != len(columns) or np.any(weights < 0) or np.isclose(weights.sum(), 0):
            raise SystemExit("가중치는 목적함수 개수와 같고, 음수가 아니며, 합계가 0보다 커야 합니다.")
    else:
        weights = np.ones(len(columns))
    weights /= weights.sum()

    # 각 Pareto 점의 정규화된 '나쁨' 점수의 가중 합이 가장 작은 점을 대표안으로 선택.
    front["selection_score"] = normalize_for_choice(front_values, directions) @ weights
    front = front.sort_values("selection_score", kind="stable")
    selected = front.iloc[[0]].copy()

    args.output.mkdir(parents=True, exist_ok=True)
    front.to_csv(args.output / "pareto_front.csv", index=False, encoding="utf-8-sig")
    selected.to_csv(args.output / "selected_design.csv", index=False, encoding="utf-8-sig")

    id_column = resolve_column(selected.columns, args.id_column) or resolve_column(selected.columns, "source_id")
    label = str(selected.iloc[0][id_column]) if id_column else str(selected.index[0])
    print(f"전체 행: {len(df)} | feasible 행: {len(feasible)} | 분석 가능 feasible 행: {len(complete)}")
    print(f"Pareto front: {len(front)}개")
    print(f"선택된 설계안: {args.id_column} = {label}, selection_score = {selected.iloc[0]['selection_score']:.6f}")
    print(f"저장 위치: {args.output.resolve()}")


if __name__ == "__main__":
    main()
