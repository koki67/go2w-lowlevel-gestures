# Plan

- Created: 2026-08-21T17:20:18+09:00
- Snapshot: 2026-08-21T17:20:18+09:00
- Status: final
- Language: ja
- Session: unavailable
- Branch: `main`（実装ブランチ `feat/adaptive-wbc-gestures` を作成予定）
- Workspace: `/home/user/ws/go2w-lowlevel-gestures`
- English translation: `2026-08-21_172018_plan_go2w-adaptive-wbc-gestures.en.md`
- Scope: 既存のno-tracking-stop系を維持したまま、追従適応型関節シーケンスと、姿勢フィードバック＋準静的WBC型シーケンスを追加する。height／rollを各3周期、fast基準で実装し、MuJoCo検証、Jetson実機評価、合格後のmain反映まで行う。

## Context and current state

- 現行制御は、実測初期関節角から固定関節姿勢へ時間ベースで補間し、LowCmdの関節PDへ渡す。実測誤差は記録・停止に使うが、軌道進行や胴体姿勢の補正には使わない。
- `live-fast-*`では追従誤差0.55 radで停止し、`live-fast-no-tracking-stop-*`では同じ指令を継続する。後者では実機側の過負荷エラーも観測されている。
- 既存の4スクリプト、特にno-tracking-stop系は削除・改名せず、指令・停止ポリシーも変更しない。
- 現在の`main`にはquick-stand、shake-off、plot保存任意化に関する未コミット差分がある。`git diff --check`、シミュレーション契約9件、Docker内全36件、`make sim-doctor`、`make sim-describe`は合格済み。生成SVGは`runs/`配下の無視対象である。
- 開発機はx86_64デスクトップ。実機Jetsonは`unitree@192.168.111.110`、リポジトリは`/home/unitree/go2w-lowlevel-gestures`。SSHパスワードは自動化へ保存せず、Jetson上のコマンドはユーザーが端末で起動する。
- Go2Wモデルは `/home/user/ws/unitree_mujoco/unitree_robots/go2w/go2w.xml`、SHA-256は `c8feaef4afdf360335727c80a826d1611950c562a3daaa5b5bfcf8b57f6859a6`。モデル質量19.126408 kg、静止重量約187.63 N、関節トルク範囲はhip/thigh ±23.7 N·m、calf ±45.43 N·m。
- LowStateには`q`、`dq`、`tau_est`、温度、mode、lost、IMU、`power_a`等があるが、Go2Wの`foot_force`は有効な実センサ値として扱わない。[Unitree LowState](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/go2/LowState_.hpp)

## Decisions and constraints

- 新規実装は次の2スクリプトとし、各スクリプトが`--gesture height|roll`を受ける。

  - `go2w_gesture_real_adaptive.py`: 追従連動のreference governorと収束ゲートを追加する関節空間制御。
  - `go2w_gesture_real_wbc.py`: 同じ適応立ち上がり後、body height／roll／pitchを閉ループ制御する準静的whole-body controller。

- 両方とも繰り返し部分はfast基準、遷移1.0秒、保持最短0.5秒、3周期とする。追従状態により実時間は延びる。startup→STANDARDは2.0秒基準、prone復帰は3.0秒基準を維持する。
- WBC版は直接トルク制御へ移行しない。100 Hzの制約付き運動学QPで関節速度・位置目標を生成し、既存500 Hz LowCmd位置PDへ渡す。`tau`指令は0のままとする。
- QPにはOSQPを使用し、Python 3.10／aarch64対応wheelがあるバージョンを固定する。[OSQP Python API](https://osqp.org/docs/get_started/python.html)、[OSQP 1.1.3](https://pypi.org/project/osqp/)
- WBCのruntimeモデルは、監査済みMJCFから抽出した最小限のリンク長、hip位置、質量、COM、関節軸、可動域、トルク範囲をリポジトリ内の明示的なパラメータとして保持する。外部`unitree_mujoco` checkoutをJetson runtime依存にはしない。
- task-space目標は既存ジェスチャー相当とする。

  - height: STANDARD基準でLOW `−0.093178 m`、HIGH `+0.076281 m`。
  - roll: STANDARD基準で左右 `±0.395469 rad`、約`±22.659°`。
  - pitchはSTANDARD時の基準姿勢、yawと水平位置は開始時基準を保持する。

- 足先センサの代わりに、低速・準静的仮定の下で `τ_contact ≈ τ_est − τ_gravity` と `τ_contact ≈ J(q)ᵀ·f_contact` を使い、各輪接触力をQP推定する。重力トルクはモデル質量・COMから計算する。
- 接触推定は制御へ無条件適用しない。Jacobian条件数、QP状態、力・モーメント平衡残差、総鉛直荷重、各輪正荷重を検査し、0.5秒連続で有効な場合だけWBCジェスチャーへ移る。無効なら適応制御でproneへ戻して非ゼロ終了する。
- belly-down→STANDARDはWBCで扱わず、adaptive版と同じ関節空間制御を使う。STANDARD保持中に四輪荷重推定を成立させ、実機ではユーザーが腹部クリア・四輪支持を目視確認してからWBCへ切り替える。
- ユーザー選択に従い、実機振幅は最初から100%、3周期とする。振幅ランプ試験は行わない。ただし異常後の自動再試行、no-tracking-stopへの自動fallback、別ジェスチャーへの自動継続は禁止する。
- 実機操作はfail closedとし、ユーザーの電源投入、車輪固定、支持、spotter、E-stop準備、確認入力が揃うまで`--live`を実行しない。

## Public interfaces

- Makeターゲットを追加する。

```text
describe-adaptive-height
describe-adaptive-roll
preflight-adaptive-height
preflight-adaptive-roll
live-adaptive-height
live-adaptive-roll

describe-wbc-height
describe-wbc-roll
preflight-wbc-height
preflight-wbc-roll
live-wbc-height
live-wbc-roll

sim-adaptive-height
sim-adaptive-roll
sim-wbc-height
sim-wbc-roll

qualify-live-adaptive-height
qualify-live-adaptive-roll
qualify-live-wbc-height
qualify-live-wbc-roll
```

- `preflight-*`はDDSとLowStateを読み取るが、Sport解放・LowCmd送信を行わない。
- `qualify-live-*`はJetson上でGit状態、aarch64、NIC/IP、build、全テスト、describe、preflightを順番に確認し、物理準備の日本語チェックリストと専用確認句を表示した後だけ対応する`live-*`を起動する。
- WBC版はSTANDARD保持中も500 Hz送信を継続しながら、別入力スレッドで `FOUR WHEELS LOADED AND BELLY CLEAR` を要求する。入力待ちで制御周期を止めない。
- 新規ログは既存trackingログを変更せず、controller種別を含むCSV＋summary JSONとする。q/dq/target/error、IMU、tau_est、mode/lost、温度、power、phase、軌道進捗、速度倍率、deadline missを記録し、WBCではtask目標・実測、推定接触力、平衡残差、QP状態・反復数・解時間も追加する。
- 資格試験ランナーは`runs/qualification/<timestamp>/`へGit SHA、実行コマンド、終了コード、端末ログ、summary、ハッシュを保存する。実行中の500 HzループではファイルI/Oを行わない。

## Final plan

1. 実装開始時にこの最終計画をグローバルへ保存し、同じ完全版を英訳する。

   - `~/.codex/memories/rollout_plans/2026-08-21_172018_plan_go2w-adaptive-wbc-gestures.md`
   - `~/.codex/memories/rollout_plans/2026-08-21_172018_plan_go2w-adaptive-wbc-gestures.en.md`

   同じ2ファイルを`docs/plans/`へ保存する。日本語版を正本とし、英訳版は構造・数値・コマンド・受入条件を省略しない。

2. 現在の未コミットシミュレーション作業を再監査する。意図しないファイルや`runs/`生成物を除外し、`git diff --check`、9件のsimulation test、Docker build、全36件、sim-doctor、sim-describeを再実行する。

3. 現在差分だけを`feat: add quick-stand and shake-off simulations`として`main`へコミット・pushする。ローカルHEAD、`origin/main`、リモート`main`のSHA一致を確認する。計画ファイルはこの既存作業コミットへ混ぜない。

4. 新しい`main`から`feat/adaptive-wbc-gestures`を作成する。以降の実装・計画ファイルはこのブランチだけへ入れ、既存no-tracking-stopスクリプトの削除・改名・閾値変更を禁止する。

5. `go2w_gesture_real.py`の所有権・停止・Sport復帰処理を再利用可能にする。

   - controller factoryまたはsubclass hookを追加し、既存wrapperの呼出し結果を変えない。
   - 新規controller用にLowStateから12脚＋4輪のtau_est、mode、lost、温度、IMU gyro/acceleration、power_v/power_aを取得する。
   - stale/nonfinite、DDS、body tilt、0.55 rad tracking stop、controlled prone return、neutral、Sport復帰を共通安全層として保持する。
   - `lost`は開始値からの増加、modeは期待値からの逸脱を停止条件にする。温度とpowerは公式のGo2W閾値がないため、今回勝手な絶対停止値を設定せず記録・summary評価対象とする。

6. 純粋計算モジュール`go2w_closed_loop_control.py`を追加する。

   - smoothstep path、phase state machine、reference governor、収束判定、運動学、Jacobian、重力トルク、接触力推定、task-space estimator、WBC QPをDDSやファイルI/Oから分離する。
   - 関節指令と実測の包絡上限をhip `0.18 rad`、thigh `0.14 rad`、calf `0.25 rad`とし、KD・重力分の余裕を残す。
   - 正規化誤差が包絡の50%以下なら予定速度、50–90%で線形減速、90%以上で進捗停止、100%以上が0.1秒継続したら最後の収束姿勢へ戻す。
   - 遷移終端は全関節が包絡の50%以内、最大関節速度`≤0.20 rad/s`を0.30秒維持したときだけ成立させる。
   - fast遷移のwall timeoutは8秒、startupは12秒、prone復帰は15秒、保持収束待ちは5秒とする。timeout時は次phaseへ進めない。
   - tau_estはモデル上限の60%で警告、75%で進捗停止、85%が0.1秒継続したらcontrolled return、100%到達で即時エラーとする。これらは物理認証値ではなく暫定アプリ保護値として明記する。

7. `go2w_gesture_real_adaptive.py`を追加する。

   - heightは既存STANDARD→LOW→HIGHを3周期、rollはSTANDARD→RIGHT→LEFTを3周期行う。
   - 目標関節姿勢は既存値を維持し、変えるのは軌道進捗とphase完了条件だけとする。
   - tracking errorまたはtau_estによる減速・停止・復帰理由をログへ残す。
   - no-tracking-stopへのfallbackを実装しない。

8. 準静的WBCを実装する。

   - 100 Hzで、generalized velocity `[base twist 6, leg dq 12]`を変数とするQPを解く。
   - 四輪中心速度ゼロを接触制約、body z/roll/pitch追従を優先task、x/y/yaw保持と既存関節姿勢を副taskにする。
   - 関節可動域、`|dq|≤1.0 rad/s`、指令加速度`≤4.0 rad/s²`、reference governor包絡をhard boundにする。
   - OSQPはwarm start、反復上限、primal/dual residual上限を固定する。unsolved、infeasible、非有限解は即座にtask進行を止め、adaptive returnへ移る。
   - LowCmdは500 Hzで最新の安全なq_refを再送し、QPは5tickごとに更新する。100 Hz solveが10 msを超える、または500 Hz deadline missが連続する場合は停止する。
   - body roll/pitchはIMU、相対heightは固定輪接触と脚運動学から荷重加重平均で推定する。絶対world heightとは主張しない。
   - `go2w_gesture_real_wbc.py`はadaptive startup→STANDARD→接触推定／目視確認→task-space 3周期→adaptive STANDARD/prone復帰の順に実行する。

9. Docker・Make・ドキュメントを更新する。

   - `numpy==1.26.4`を維持し、`scipy==1.13.1`と`osqp==1.1.3`を固定してx86_64／aarch64 buildを検証する。[SciPy 1.13.1](https://pypi.org/project/scipy/1.13.1/)
   - 2つの新規hardware wrapper、共通制御モジュール、simulation runner、資格試験runnerをimageへ含める。
   - READMEでは「adaptive joint-space」と「quasi-static kinematic WBC」を区別し、後者が直接トルク型dynamic WBCではないこと、100%実機試験がユーザー選択であることを明記する。
   - 既存Makeターゲットとの互換性とno-tracking-stopの説明を維持する。

10. MuJoCo用の`simulation/go2w_closed_loop_sequence_sim.py`を追加する。

    - hardwareと同じ純粋制御kernelを使用し、DDS所有権部分だけsimulation harnessへ置き換える。
    - adaptive/WBC×height/rollを各3周期実行する。
    - 通常spawnに加え、非対称prone、腹部荷重が大きいproneを準備phaseで作り、その実測状態から制御を開始する。
    - 外部MJCFやsource checkoutを変更せず、一時sceneだけを使用する。
    - simulator ground truthのbase pose、接触、actuator forceは評価ログにだけ使い、hardware制御入力へは混ぜない。

11. テストを追加する。

    - reference governorの通常進行、減速、停止、backoff、timeout、収束ゲート。
    - q_ref包絡、関節速度・加速度・可動域。
    - 解析Jacobianと有限差分の一致。
    - 既知接触力から生成したtau_estの復元、重力補償、特異姿勢・負荷不整合の拒否。
    - task-space height／rollの方向・目標値、四輪接触速度残差。
    - OSQP infeasible／timeout／非有限出力時のfail-closed動作。
    - 500 Hz loop中にファイルI/Oがないこと。
    - adaptive/WBCのheight／roll各3周期、prone復帰、neutral、Sport復帰。
    - 既存36件と既存4wrapperの出力・停止ポリシーが変わらないこと。
    - Jetson qualifierが`--live`なしではLowCmdを送らず、各失敗段階で後続処理を実行しないこと。

12. デスクトップ資格試験を行う。

    - `git diff --check`
    - Docker build、全単体・契約テスト、`pip check`
    - 全describe/preflight dry-run
    - MuJoCoでadaptive/WBC×height/roll×3初期条件
    - task追従、接触推定残差、tau_est比、tracking error、QP timing、return姿勢をsummary化する。
    - simulation PASSと実機資格を明確に分離する。

13. 実装を3つの論理コミットに分ける。

    - `feat: add adaptive closed-loop gestures`
    - `feat: add quasi-static whole-body gestures`
    - `test: add closed-loop gesture qualification`

    clean cloneでDocker・テスト・describeを再実行後、`feat/adaptive-wbc-gestures`をoriginへpushし、リモートSHAを記録する。

14. Jetson実機評価はユーザーがロボットを起動した時点で開始する。

    - ユーザーがJetson端末で`/home/unitree/go2w-lowlevel-gestures`へ移動する。
    - dirty worktreeなら停止し、stash/reset/削除を自動実行しない。
    - feature branchをfetch／switchし、デスクトップで評価したSHAと一致させる。
    - 実行順はadaptive-height、adaptive-roll、WBC-height、WBC-rollとする。
    - 各実行前にユーザーがロボットの電源、belly-down静止、車輪固定、支持具、spotter、E-stop、LowCmd単独所有を確認する。
    - ユーザーが対応する`make qualify-live-*`を起動し、スクリプトがbuild/test/describe/preflightを自動実行する。
    - preflight合格後に専用確認句を要求し、WBCではSTANDARD到達後に四輪荷重推定値を表示して腹部クリア確認をもう一度要求する。
    - 各ジェスチャーは100%振幅、最初から3周期で実行する。
    - 一件でも異常、firmware error、E-stop、controlled-return失敗、Sport復帰未確認があれば残りの全liveを中止する。no-tracking-stopで再試行しない。

15. 4ケースすべてが合格した場合だけ、デスクトップでfeature branchを`main`へ`--no-ff` mergeし、全テストを再実行してpushする。ローカルHEAD、`origin/main`、GitHub remote SHAを照合する。feature branchは削除しない。

## Validation

### 構造・ソフトウェア

- 既存未コミット作業が独立コミットとしてmainへ入り、今回のfeature commitと混在しない。
- 既存4スクリプトと全既存Makeターゲットが残る。
- x86_64とJetson aarch64のDocker build、`pip check`、全テストが合格する。
- QP solveのデスクトップp99が5 ms未満、Jetson p99が8 ms未満で、500 Hz publication deadlineを継続的に満たす。

### MuJoCo

- adaptive/WBCのheight／rollが各3初期条件で3周期完了し、proneへ復帰する。
- adaptive版はq_ref包絡を超えず、0.55 rad watchdogへ到達しない。
- WBC版はheight誤差`≤0.015 m`、roll/pitch誤差`≤2°`を各hold終端で満たす。
- 四輪接触速度残差`≤0.01 m/s`、接触力推定平衡残差`≤15%` of body weightを満たす。
- モデル関節・トルク制約、tilt、solver、state freshnessの違反がない。
- これらはsimulation qualificationであり、物理合格とは表現しない。

### 実機

- 各runのGit SHA、preflight、ユーザー確認、liveログ、summary、終了コードが保存される。
- 4ケースすべてで3周期を完了し、firmware error、E-stop、tracking/tau/tilt/mode/lost/watchdog違反がない。
- WBC版は四輪支持推定が有効で、ユーザーが腹部クリア、意図した屈伸／左右roll、車輪滑り・浮き・床衝突なしを目視確認する。
- 全runでcaptured prone復帰、LowCmd writer close、startup Sport service復帰が確認される。
- 実機合格前はmainへmergeしない。

## Acceptance criteria

- `go2w_gesture_real_adaptive.py`と`go2w_gesture_real_wbc.py`が、height／rollの両方を明示選択できる。
- adaptive版が実測追従に応じて軌道を減速・停止・復帰し、時間だけで次phaseへ進まない。
- WBC版がIMUと運動学によるbody poseフィードバックを行い、tau_est＋Jacobianによる四輪荷重推定を検証してからtask-space motionへ移る。
- 既存no-tracking-stop系が保持されるが、新規制御から自動使用されない。
- シミュレーション合格、Jetsonソフトウェア合格、実機物理合格、Git公開状態が別々に記録される。
- 4つの100%・3周期liveが全合格した場合のみmainへ反映される。

## Risks / cautions

- `tau_est`は直接の足先力センサではなく、摩擦・重力モデル・機体差・温度・Jacobian条件数の影響を受ける。推定不整合時はWBCを実行しない。
- 腹部接触を直接測るセンサはない。adaptive startup、総荷重整合性、運動学残差、ユーザー目視を組み合わせるが、任意の初期配置からの復帰を保証しない。
- 今回のWBCは準静的・運動学型であり、直接トルクを最適化するdynamic WBCではない。高速・跳躍・接触切替には適用しない。
- 100%振幅・3周期を初回から行う選択は、段階的振幅試験よりリスクが高い。通常watchdogを維持し、異常時は即中断して再試行しない。
- simulatorのtau_estはactuator force、実機のtau_estは推定値であり一致は保証されない。MuJoCo合格だけで物理安全を主張しない。
- WBC solverやSciPy/OSQPのaarch64 timingはJetson上で再測定し、周期条件を満たさなければliveへ進まない。
- 現在の未コミット作業、生成SVG、Jetson上のdirty workを上書き・stash・reset・削除しない。
