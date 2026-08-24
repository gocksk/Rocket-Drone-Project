# 미구현 목록 (STUBS)

기준: **ICD0-008** · 단계: **P0 완료 시점**

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

### `modules/geom.py` — 전부 스텁 (P2 · P6)
| 함수 | 상태 | 없는 것 |
|---|---|---|
| `hull(dv)` | 스텁 | `S_ref` `S_wet` `l_body`(←C8) `r_body` 핀 기하 전부 자리표시 |
| `layout(dv, hull, dims)` | 스텁 | 배치 규칙(카메라→센서→배터리→FC/ESC), `x_parts` 전부 0, `arm_rotor` |
| `check_fit(…)` | 스텁 | g6(내장 여유) · g7(클리어런스) 판정 없음 |

### `modules/aero.py` — 전부 스텁 (P2)
| 항목 | 상태 | 없는 것 |
|---|---|---|
| `F_drag(V, α)` | 스텁 | 성분별 항력 빌드업 · 압축성 보정 |
| `CL(α)` | 스텁 | **트림 연립(§4.1)의 전제**. 0 으로 두면 P3 완료판정이 성립하지 않는다 |
| `CN_alpha` · `x_cp` | 스텁 | Barrowman |
| `CN_alpha_fin` | 스텁 | STRC 핀 하중용 |
| `q_cr` | 정의식 | 구현됨 (0.5ρV²) |

### `modules/prop.py` — 대부분 스텁 (P3 · P4)
| 함수 | 상태 | 없는 것 |
|---|---|---|
| `U_ocv(SOC)` | 정의식 | 3점 선형. 상수 `U_cell_*` 가 TBD |
| `R_pack` · `U_eval` | 정의식 | `k_Rpack` TBD |
| `motor_elec(…)` | 정의식 | §4.3 수식은 확정. `R_mot`·`I0` 를 주는 회귀가 없다 |
| `motor_regression(m_mot, kv)` | **스텁** | ICD §8 A-2 회귀 미수행. 계수도 형태(멱함수)도 잠정 |
| `build_map(dv, air)` | **스텁** | BEMT 미구현. `CT(J)`=`CP(J)`=0, g1 판정 없음. `m_prop` 만 정의식 |
| `solve_point(…)` | **스텁** | 트림 2×2 뉴턴 + 토크 평형 rpm 탐색 둘 다 없음 |
| `size_motor(…)` | **스텁** | 이분법 없음. `m_mot`=0, g2·g3 판정 없음, 활성조건 `"stub"` |
| `evaluate(…)` | **스텁** | V_max 이분법 없음 → **C3(가중치 33.7%)** · C6 가짜 |

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

---

## 3. 지금 깨져 있는 것

- **`validation/wght/` 16종 테스트가 안 돈다.** `common/srl` 과 ICD0-007 형상
  (`StrcOut.m_str`, `_iterate` 의 `strc_of`)에 묶여 있다. **P1 의 작업 범위**다
  (`strc_stub.py` 반환 형태 · `test_wght.py` 의 `pipeline()` → `main.preprocess()`).
  따라서 `.github/workflows/ci.yml` 의 두 번째 스텝이 P1 전까지 빨간불이다.
- **`README.md` 의 파일 표가 ICD0-007 기준**이다 (`prop_a` `prop_b` `common/srl`).
  담당자 배정 정보가 섞여 있어 임의로 고치지 않았다.

## 4. 참고

ICD0-007 기준의 GEOM·AERO·PROP-A/B·STRC·STAB·COST 구현은 커밋 `d801929` 에 남아
있다. P2·P6 에서 수식을 이식할 때 참고한다 (설계변수와 g 번호가 달라 그대로는 못 쓴다).
