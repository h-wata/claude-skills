---
name: poll-sonnet-workers
description: tmux で動いている Sonnet 4.6 ワーカー (W1-W3) の状態を一括チェックし、permission prompt 待ち / 長時間停止 / コンテキスト枯渇 / 完了報告未読を検出する skill。Sonnet は auto mode が無いため放置すると prompt 待ちで止まる。Dispatcher が「ワーカーの状況は？」「止まっていない？」「進捗どう？」と言われたとき、または並列タスク発注後 3-5 分経過したときに使う。ros-agents tmux session 前提。
allowed-tools: Bash
---

Sonnet ワーカーは Anthropic の auto mode 対象外なので、permission 確認プロンプトや
unhandled state で **無音で停止** する。Dispatcher が手動で状態を覗かないと
気付けない。この skill は `tmux capture-pane` を全 worker pane に走らせて、
1 度に状況サマリを返す。

## なぜ必要か

2026-05-21 の Wave 3 実装で、W1/W2 が以下のような場面で無音停止した:

- `pip install -e .` の permission prompt 待ち
- `/safe-pathspec-commit` の skill ロード許可 prompt
- Edit ツールの file path 許可 prompt
- 長時間 thinking (5 分以上同じ状態)
- Context 残量 20% 切ったまま新タスク投入

Dispatcher は他の作業（cross-review 待機 / dashboard 更新等）で目が離れる。
3-5 分おきに pane を覗くのは手作業で面倒、見落としも起きる。

## 使うタイミング

- Dispatcher が「ワーカーの状況は？」「止まってない？」と user に聞かれた
- 並列タスクを発注して 3-5 分経過、まだ完了報告が来ていない
- 1 つの worker が長い処理（テスト走らせ / 大量ファイル編集）を始めた直後
- `/loop` で定期実行したい場合の 1 サイクル分として
- セッション開始直後、各 worker が以前の状態を維持しているか確認したい

## 前提

- tmux session `ros-agents` が動いている
- pane 構成 (tmux-multi-agents の標準):
  - Pane 0: Dispatcher
  - Pane 1: Worker 1 (Claude Sonnet)
  - Pane 2: Worker 2 (Claude Sonnet)
  - Pane 3: Worker 3 (Claude Sonnet)
  - Pane 6: Worker 4 (Codex)
- 各 Claude pane の bottom bar に context 残量と model 名が表示されている

## 手順

### Step 1: 全 Claude worker pane を一括 capture

```bash
for n in 1 2 3; do
  echo "=== Pane $n (Worker $n) ==="
  tmux capture-pane -t ros-agents:0.$n -p | tail -20
  echo
done
```

各 pane の **最後 20 行** を取る。それ以上だと出力が冗長、それ未満だと現在の
prompt や状態行を取り逃す可能性。

### Step 2: 異常パターンを検出

以下のキーワードを grep で軽くチェック。手動で目視でも可。

#### A. Permission prompt 待ち
```bash
# 検出パターン
grep -E "Do you want to proceed\?|requires confirmation|Press enter to confirm"
```
- `Permission rule Bash(...) requires confirmation for this command` → 命令名と option 番号を控える
- `Do you want to proceed? ❯ 1. Yes / 2. ... / 3. No` → Codex 風
- `Press enter to confirm or esc to cancel` → 待機状態

**対処**: tmux で `send-keys` で option 番号送信、または該当タスクの
permission allowlist を見直す

#### B. 長時間 thinking
```
✶ Sprouting… (X 秒) / · Whirring… (X 秒) / ✽ Topsy-turvying…
```
カッコ内秒数が **5 分 (300s) 以上** で動きなし → loop / 無限再試行の疑い

**対処**:
- 5-10 分待って再確認、それでも進まなければ `Esc` + メッセージで状況確認
- 同じ tool を 3 回以上失敗していたら別アプローチを促す指示を送る

#### C. コンテキスト枯渇
bottom bar の右側に `◔ NNN,NNN (XX%)` が出る。
- XX% > 30%: 健全
- XX% 20-30%: 注意 (次タスク投入前に `/compact`)
- XX% < 20%: 危険 (`/clear` 推奨、現タスク完了まで様子見)

**対処**:
- 20% 切ったら新タスク投入前に worker に `/compact` を送る
- 10% 切ったら現タスク完了報告後に `/clear` を送る

#### D. モデル取り違え
bottom bar の左に `✱ Sonnet 4.6` 等が出る。**期待と違うモデル**で動いていないか確認:
- Dispatcher が `/model sonnet` を送り忘れた → `/model sonnet` 送信
- `auto mode unavailable for this model` という警告 → Sonnet なので想定内、無視

#### E. 完了報告未読
最終行付近に `TASK-XXX 完了` や `PR #N` のような文字列があるのに、Dispatcher 側で
まだ ack していない可能性 → reports/worker{N}_report.yaml の mtime を比較

```bash
ls -la /home/gisen/work/tmux-multi-agents/queue/projects/*/reports/worker{1,2,3}_report.yaml \
  2>/dev/null
```
mtime が **直近 10 分以内** で Dispatcher の dashboard 更新時刻より新しければ
未読の疑い。

#### F. Codex (W4) も軽くチェック (オプション)
```bash
tmux capture-pane -t ros-agents:0.6 -p | tail -15
```
- bottom bar `Context XX% left` で確認
- `Press enter to confirm or esc to cancel` 系の permission prompt
- `Working (Xm Ys)` の長時間 thinking

Codex は auto mode あるので Claude より停止頻度低いが、permission prompt
パターンは別系統（許可リスト構築が個別）。

### Step 3: サマリ出力

検出結果を 1 ブロックで報告:

```
=== Sonnet Workers Status (HH:MM JST) ===

Worker 1 (Pane 1):
  - State: 待機中
  - Context: 74%
  - Last activity: TASK-187 完了報告 15:10
  - Anomalies: なし

Worker 2 (Pane 2):
  - State: ⚠️ permission prompt 待ち
  - Context: 81%
  - Last activity: /safe-pathspec-commit 許可確認 (15:25)
  - Action needed: tmux send-keys "2" Enter (Yes, allow)

Worker 3 (Pane 3):
  - State: 待機中
  - Context: 95%
  - Last activity: なし (idle 全 session)
  - Anomalies: なし

Worker 4 (Pane 6, Codex):
  - State: スタンバイ
  - Context: 89% left
  - Anomalies: なし

合計: 異常 1 件 (W2 permission 待ち)
```

異常 0 件のときは "全 worker 健全、異常なし" だけ書いて短く済ませてよい。

## 異常検出パターン早見表

| 検出文字列 | 状態 | 対処 |
|---|---|---|
| `Do you want to proceed?` | Permission 待ち | tmux で option 番号 + Enter |
| `requires confirmation for this command` | Permission 待ち | tmux で 2 (always allow) を推奨 |
| `Press enter to confirm or esc to cancel` | Codex permission 待ち | `p` (always) または `y` (this time) |
| `Sprouting… (>300s)` | 長時間 thinking | Esc + 状況確認 |
| `◔ ... (<20%)` | Context 枯渇近い | `/compact` or `/clear` 提案 |
| `auto mode unavailable for this model` | Sonnet 表示 | 想定内、無視 |
| `not in a mode` | tmux 入力モード異常 | 該当 pane に短いメッセージで起こす |

## /loop での定期実行

長時間並列タスクを監視したいとき:

```
/loop 5m /poll-sonnet-workers
```

5 分おきに走らせて、異常 0 件のときは output 短くしておくと dispatcher の
context を食いにくい。

ただし worker が活発に動いているとき (1m 以内に完了 / 5m 以内に PR push 等)
は `/loop` は **過剰**。ad-hoc 呼び出しで十分。

## 出力ガイドライン

- **異常 0 件のとき**: 3-5 行で終わらせる
- **異常 1 件以上のとき**: 該当 worker のみ詳細、それ以外は 1 行サマリ
- 文字数を増やす意味は無い（dispatcher の context を食うだけ）
- 「次に何すべきか」を 1 行で必ず添える（Action needed）

## やってはいけないこと

- worker pane に勝手に `send-keys` で介入しない（Dispatcher の判断を待つ）
- 完了報告 yaml を勝手に消費しない（Dispatcher が読む）
- 全 pane の全行を pretty-print しない（context を浪費）
- 異常検知だけ報告して "様子見が必要" と曖昧に終わらない（具体的 action を出す）
