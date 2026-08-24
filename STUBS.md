# 미구현 목록 (STUBS)

기준: **ICD0-008** (원문 사본: `docs/ICD0-008.md`) · 단계: **P3 완료 시점**

이 문서는 "아직 진짜가 아닌 것"의 유일한 목록이다. 코드에서 `[스텁]` 또는 `# TBD` 를
지울 때 여기서도 지운다. 그럴듯한 물리값을 지어내 채우는 것이 이 프로젝트에서 가장
비싼 실수이므로, 값이 없으면 없다고 적는다.

표기
- **스텁** — 함수는 있고 시그니처는 맞지만 반환값이 자리표시 숫자다. 물리적 근거 없음.
- **TBD** — 상수 값이 미확정. 붙은 숫자는 임시이며 근거 없음.
- **정의식** — 물리 모델이 아니라 정의라서 구현이 끝났으나, 쓰는 상수가 TBD 인 경우.

---

## 1. 모듈별 미구현

### `modules/atm.py` — **스텁 없음**
`run(h)` 는 실제 구현이며 검산까지 통과한다 (`python -m modules.atm`).
h=0 에서 ρ=1.2250 · a=340.29 · μ=1.789e-5. 기존 `common/atm.py` 를 ICD0-008
시그니처로 옮긴 것이다.

### `modules/geom.py` — `hull` 구현 완료, 나머지 스텁 (P6)
| 함수 | 상태 | 없는 것 |
|---|---|---|
| `hull(dv)` | **스텁 아님** | 손계산 대조 통과 (`python -m modules.geom`) |
| `layout(dv, hull, dims)` | 스텁 | 배치 규칙(카메라→센서→배터리→FC/ESC), `x_parts` 전부 0, `arm_rotor` 가 `f_mount` 에 무반응 |
| `check_fit(…)` | 스텁 | g6(내장 여유) · g7(클리어런스) 판정 없음 |

`hull` 은 ICD §5.1 대로 포드를 빼고 노즈+원통+핀만 낸다. 포드 치수는 모터 질량의
함수라 ⓪에서 알 수 없다. 형상 계수 `f_nose` `k_nose` `lam_fin` `tc_fin` `k_thk` 는 TBD.

### `modules/aero.py` — 구조 완료, 계수 TBD (P2 완료)
성분별 빌드업 · Prandtl-Glauert 압축성 보정 · Barrowman · 크로스플로 · 풍축 변환까지
표준식으로 구현했다 (`python -m modules.aero`). 반응 방향은 믿을 수 있고, **절대값은
아래 계수가 확정되기 전까지 보증되지 않는다.**

| 항목 | 상태 | 남은 것 |
|---|---|---|
| `F_drag(V, α)` · `CD0` | 구현 | 계수 `k_base` `k_int` `k_cal` TBD |
| `CL(V, α)` | 구현 | ICD 는 `CL(α)` — 축력항의 V 의존 때문에 넓혔다 (docs §11-3) |
| `CN_alpha` · `x_cp` | 구현 | `k_cp_nose` TBD |
| 크로스플로 | 구현 | `Cd_c` `eta_cf` `k_side` `k_finproj` TBD |
| 포드 항력 | 식은 있음, 배선 없음 | `F_drag(V, α, pod=(d_pod,l_pod))` 로 받는다. 포드 치수를 아는 ① `size_motor` 가 오기 전까지 실효값 0 — **P4 에서 배선** (docs §11-2) |

### `modules/prop.py` — `build_map`·`solve_point` 구현 완료 (P3), 나머지 스텁
| 함수 | 상태 | 남은 것 |
|---|---|---|
| `U_ocv(SOC)` | 정의식 | 3점 선형. 상수 `U_cell_*` 가 TBD |
| `R_pack` · `U_eval` | 정의식 | `k_Rpack` TBD |
| `n_tip_limit` · `M_tip` | 구현 | — |
| `J_cruise` · `pd_prop_min` | 구현 | §2 하한 규칙(계수 1.25). DOE 범위 설정용 |
| `build_map(dv, air)` | **구현** | BEMT + Prandtl 팁손실. 블레이드 규약 7종이 TBD (docs §11-15). **앵커 보정 없음 → 호버 추력 과대평가** (§8 A-5) |
| `_trim` · `solve_point(…)` | **구현** | 트림 2×2 뉴턴 + rpm·kv 이분법. 243 케이스 100% 수렴 |
| `motor_elec(…)` | 정의식 | §4.3 수식은 확정 |
| `motor_regression(m_mot, kv)` | **스텁** | ICD §8 A-2 회귀 미수행. `a_R` 은 실측 1점에 자릿수만 맞춘 값이라 **kv·I 결과를 믿으면 안 된다** |
| `size_motor(…)` | **스텁** | P4. 이분법 없음, `m_mot`=0, g2·g3 판정 없음, 활성조건 `"stub"` |
| `evaluate(…)` | **스텁** | V_max 이분법 없음 → **C3(가중치 33.7%)** · C6 가짜 |

⚠ `build_map` 이 설계점당 약 **149 ms** 다. ⓪ 에서 가장 무겁고 DOE 규모의 지배 항목이 된다.

### `modules/thrm.py` — 전부 스텁 (P4)
`motor_rise(…)` — 덩어리 열용량 ODE 미구현. `T_peak = T_amb` 로 나가므로
열 여유가 항상 최대로 보인다. **g3 이 사실상 죽어 있다.**

### `modules/miss.py` — 전부 스텁 (P5)
| 함수 | 상태 | 없는 것 |
|---|---|---|
| `integrate(…)` | 스텁 | 고정 스텝 전진 없음. 이력 배열이 길이 1 |
| `required_energy(…)` | 스텁 | 이분법 없음. `E_batt`=0 → `m_batt`=0. 활성조건 `"stub"` |
| `achieved_range(…)` | 스텁 | 커널이 스텁이라 **C2** 가짜 |

### `modules/strc.py` — 스텁 (P6)
`run(…)` — 슬라이서 회귀·응력 검산 없음. g5 판정 없음.
`W_str = 0.30 × MTOW` 라는 **가짜 선형 기울기**를 쓴다 (`_STUB_SLOPE`).
0 을 반환하면 응답 질량이 MTOW 에 반응하지 않아 ① 수렴 루프가 무의미해지므로
배관 확인용으로 넣은 값이며, **물리적 근거가 전혀 없다.**

### `modules/wght.py` — **스텁 아님** (한 곳 제외)
`_iterate` · `converge` · `mass_props` 는 실제 구현이며 검증된 자산이다.
P0 에서는 ICD §8 C-3 계약 일반화(`strc_of` → `resp_of`)만 했고 상태머신의
분기 조건은 한 줄도 바뀌지 않았다.

| 함수 | 상태 | 없는 것 |
|---|---|---|
| `growth_split(history, …)` | **스텁** | 성장계수 분해 S_i (구조·모터·배터리) 미산출. P7 |

### `modules/stab.py` — 전부 스텁 (P6)
`run(…)` — SM · M_dist · M_ctrl · alpha_max 전부 0. g8·g9 판정 없음 → **C5** 가짜.
⚠ `M_dist` 를 상수로 두면 안 된다 (§4.4). 구현 시 `q·S_ref·CN_α·Δα·|x_cp−x_cg|`.

### `modules/cost.py` — 전부 스텁 (P6)
`run(…)` — 단가 계수가 전부 TBD 라 항목별 0. **C7** 가짜.

### `main.py`
| 항목 | 상태 | 내용 |
|---|---|---|
| `U_bus` | **스텁** | §4.5 의 `U_eval` 은 `I_dash`·`R_pack` 순환이라 I·R 항을 뺀 값으로 뒀다 |
| `dims` | **스텁** | 부피→치수(L·W·H) 분해 미구현. `layout` 이 스텁이라 아직 무해 |

---

## 2. 값 미확정 (`constants.py` 의 `# TBD`)

ICD §8 B 에 대응한다. 붙은 숫자는 전부 임시다.

- **B-1 물성·단가** — `rho_pack` `wh_pack` `rho_mot` `k_Rpack` `k_mprop`
  `c_mot_krw` `c_batt_krw` `c_prop_krw` `c_esc_krw` `c_filament_krw`
- **B-2 설계변수 범위** — `d_body` `lambda_body` `S_fin` `x_fin` `AR_fin`
  `d_prop` `pd_prop` `k_E` `k_mot` 범위 전부 (constants 가 아니라 DOE 설정)
- **B-3 임무·요구** — `W_pl` `R_dash_min` `MISSION_PROFILE`(t_seg) `k_trans`
  `t_trans` `c_rate_max` `e_spec` `dt_miss`
- **B-4 안정 판정** — `k_ctrl` `alpha_dot_req` `d_alpha`(Δα)
- **A-2 모터 회귀** — `a_R` `b_R` `c_R` `a_I0` `b_I0` `pm_mot`
- **A-3 열** — `T_limit` `T_amb` `c_mot` `ar_mot` `Nu_C_*` `Nu_m_*`
- **§8 C-4 잔차 하한** — `resid_floor` `delta_r` (`dt_miss` 확정 후 재측정)
- **배터리 셀** — `U_cell_full` `U_cell_nom` `U_cell_cut`
- **소음 앵커** — `V_tip_ref` `T_ref` `N_ref` `r_obs` (`N_ref` 는 6 dB 가 걸린 미확인 항목)

`# ICD외` 표시 상수 — ICD §3 표에 없는데 코드가 필요로 해서 추가했다. 통합 담당 확인 대상:
`N_rot` `g` `h_miss` `SPL_ref` `r_ref` `r_obs` `V_tip_ref` `T_ref` `N_ref`
`d_clr` `k_esc_margin` `c_filament_krw`
— P2 에서 추가: `f_nose` `k_nose` `lam_fin` `tc_fin` `k_thk` (GEOM 형상 규약) ·
`k_base` `k_int` `k_cal` `k_form` `M_pg_max` (AERO 빌드업) · `Cd_c` `eta_cf` `k_side`
`k_xn` `k_finproj` (크로스플로) · `CN_nose` `k_cp_nose` (Barrowman)
— P3 에서 추가: `k_pitch_margin` (§2 하한 규칙 계수, 원문 1.05→1.25) ·
`B_blade` `r_hub_ratio` `c_over_D` `cl_alpha_2d` `cl_max_2d` `cd0_2d` `k_cd_2d`
(BEMT 블레이드 규약) · BEMT·트림 수치 파라미터 12종

전부 `docs/ICD0-008.md` **§3.9** 에 등재돼 있다.

---

## 3. 지금 깨져 있는 것

- **`validation/wght/test_wght.py` 16종 중 1종 실패 — TEST 13(c).**
  `arm_rotor` 를 늘렸을 때 `J_xx` 가 커지는지 보는 항목인데, 전제인 "arm_rotor 가
  커진다"가 성립하지 않아 거기서 멈춘다. `geom.layout` 이 스텁이라 `f_mount` 를
  무시하고 `arm_rotor = 1.0` 을 고정 반환하기 때문이다 (**GEOM 사유, WGHT 아님**).
  `wght.mass_props` 자체는 `Σm·r²` 로 정확히 반응하는 것을 단독 확인했다
  (r=0.10/0.20/0.40 → J_xx=2.000/8.000/32.000 g·m², 이론값 일치).
  → **`geom.layout` 구현(P6)이 끝나면 자동으로 풀린다.** P2 의 `geom.hull` 만으로는
  안 풀린다 — `arm_rotor` 는 배치(②) 소관이기 때문이다.

  **이 실패는 의도적으로 남겨 둔 것이다.** 테스트를 고치거나 skip 표시하지 않는다 —
  이 한 줄이 "GEOM 배치가 아직 스텁"이라는 사실을 가리키는 알람 역할을 하고,
  P6 가 끝나면 알람이 스스로 꺼지는 것이 완료 판정이 된다.
  그때까지 `.github/workflows/ci.yml` 의 WGHT 검증 스텝은 빨간불이다.
- **`measure_noise.py` 는 응답 질량 중 구조 항만 잰다.** ICD §8 C-4 는 측정 대상을
  전체 응답 질량(구조+모터+배터리)으로 넓히라고 하는데 PROP·MISS 가 스텁이다.
  P4·P5 와 `dt_miss` 확정 후 다시 재야 한다.
- **`README.md` 의 파일 표가 ICD0-007 기준**이다 (`prop_a` `prop_b` `common/srl`).
  담당자 배정 정보가 섞여 있어 임의로 고치지 않았다.

## 4. ICD 승인 대기

`docs/ICD0-008.md` §11 에 개정 15건이 근거와 함께 모여 있고, **전부 승인되어 본문에
반영**됐다 (§11 머리말의 상태표 참조). 그중 아직 **미결**로 남은 항목:

- §11-2 포드 항력 **호출부 배선** — P4 `size_motor` 가 포드 치수를 낼 때 붙인다
- §11-6 질량 분해표 조립의 소유권 (WGHT vs 런처)
- §11-7 **ESC 질량이 ICD 어디에도 없다** — 분해표에서 빠져 있다
- §11-15 BEMT 블레이드 규약 계수 7종 TBD
- **`pd_prop` 상한 미정** — 하한(1.466)만 정했다. 피치가 크면 호버·저속에서
  블레이드가 실속하므로 상한은 P4(호버 열)·P5(호버 에너지)에서 나와야 한다

## 5. 참고

ICD0-007 기준의 GEOM·AERO·PROP-A/B·STRC·STAB·COST 구현은 커밋 `d801929` 에 남아
있다. P2·P6 에서 수식을 이식할 때 참고한다 (설계변수와 g 번호가 달라 그대로는 못 쓴다).
