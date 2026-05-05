---
name: mcp-smoke-via-claude-p
description: 任意の MCP server に対して `claude -p --permission-mode bypassPermissions --output-format json` で 5 case の自動 smoke を回す。LLM が tool を選んだか、permission_denials が 0 か、tool 引数と結果が期待通りかを JSON parse で機械検証。MCP smoke の人手介入を避けたいときに。
allowed-tools: Bash, Read, Write
---

# mcp-smoke-via-claude-p

`claude -p` を使って MCP server を 5 ケース smoke する自動化パターン。
mesh-mem の Issue #29 / TASK-175（2026-05-04）で確立。`claude` 対話 UI が
不要なので CI からも呼べる。

## いつ使うか

- 新しい MCP server を作って動作確認したいとき
- 既存 MCP server を upgrade した後の回帰テスト
- 複数 MCP server を毎日チェックする健全性監視

## 重要な注意：bypassPermissions が必須 + 安全境界

`claude -p` (non-interactive) モードでは UI 承認ダイアログが出せない。
よって `--permission-mode bypassPermissions` を渡さないと、最初の MCP
tool 呼び出しが `permission_denials` に積まれて LLM が「許可が必要」と
返答して終わる。これは `-p` モードの設計どおりの挙動だが、自動 smoke では
必ず bypass すること。

⚠️ **bypassPermissions は危険な flag**。LLM が呼びうる **全ての** MCP tool
が無条件で実行可能になる。次のすべてを満たす環境でのみ使うこと：

1. **trusted MCP server に限定** — 自分でビルド / source 確認した server、
   または信頼できる供給元のもの。任意の MCP server に対してこの skill を
   そのまま流用しない。
2. **副作用を限定するテスト project / namespace を必ず指定** — `mcp-smoke`
   等の専用 project で、既存データに触れない / 削除しても困らないスコープ
   を作る。本番 project / 本番アカウントを直接叩かない。
3. **書き込み権限が production リソースに到達しないアカウント** — production
   DB / production credential を持つ MCP server には使わない。dev / staging
   / 専用 sandbox で実施する。
4. **MCP server の tool 一覧を事前に把握** — `tools/list` で expose されている
   tool を確認し、データ削除 / 課金 / 通知送信 / 外部 API 呼び出し系の tool
   が含まれる場合は smoke の prompt が誤って tool 起動しないか追加レビュー。

これらが揃わないなら、`bypassPermissions` ではなく interactive mode で 1 回
ずつ承認しながら回すか、別の検証方法（unit test、Python から fastmcp Client
を使う、等）を選ぶ。

## 入力（必ず確認すること）

| 項目 | 例 | 用途 |
|------|------|------|
| `MCP_NAME` | `mesh_mem` | `claude mcp list` で識別される名前 |
| `MCP_BIN` | `/home/gisen/work/mesh-mem/.venv/bin/mesh-mem-mcp` | MCP server 実行ファイル |
| `MCP_ENV` | `ZENOH_CONNECT=tcp/127.0.0.1:7447` 等 | 登録時に渡す env |
| `TEST_PROJECT` | `mcp-smoke` | 副作用を限定するためのテスト名前空間（save / search / delete のスコープ） |
| `EXTERNAL_VERIFY_CMD` | `mesh-mem search '' --project mcp-smoke --limit 5` | 直接 CLI で削除確認するためのコマンド |

## 適用範囲（scope）

この標準 5 case は **CRUD 型 MCP server**（create / read / status / delete に
近い tool を持つ server）を前提に設計されている。具体的には：

- 「データを保存する tool」「保存したデータを検索する tool」「全体状態を返す
  tool」「ID 指定で削除する tool」が一通り揃っている server
- 副作用が同じ namespace 内で完結する（外部 API 呼び出し / メール送信 / 課金
  などが起きない）

**該当しない server には流用しない / 構造を読み替えること**：

- read-only / observability 系（例: ログ検索だけ、メトリクス取得だけ）→ 5 case
  の save / delete を該当する read tool に置き換える、または case 数を減らす
- アクション実行系（例: deploy 起動、コマンド実行、外部 API 呼び出し）→ smoke
  に向かない。bypassPermissions の安全境界が崩れる
- 双方向通信 / streaming（例: 長期 subscribe）→ `claude -p` の単発呼び出しでは
  意味のあるテストにならない

迷ったら **server の tool 一覧を最初に列挙** し、CRUD に対応するか確認する。

## 5 ケース構成（CRUD 型 server 向け）

| # | tool | 目的 | 検証 |
|---|------|------|------|
| 1 | (none) | `claude mcp list` で `✓ Connected` | grep "Connected" |
| 2 | save / create 系 | LLM が保存 tool を選び、引数を渡す | `permission_denials = 0`、ID 抽出 |
| 3 | search / read 系 | LLM が検索 tool を選び、Case 2 の結果を取れる | result に Case 2 の ID 含む |
| 4 | status 系 | LLM が状態 tool を選び、期待フィールドが返る | result に version / pc_id 等 |
| 5 | delete / remove 系 | LLM が delete tool を 32-char ID で呼ぶ | external verify で 0 件 |

mesh-mem 以外の MCP では tool 名が違うので prompt 文言を調整する。
CRUD scope に収まる範囲で構造（5 case + permission_denials 検証）は流用可。

## 標準呼び出しテンプレート

```bash
claude -p \
  --permission-mode bypassPermissions \
  --output-format json \
  "<natural-language prompt>" \
  > /tmp/case_N.json 2>&1

# parse
python3 -c "
import json
d = json.load(open('/tmp/case_N.json'))
print('result:', d.get('result','')[:500])
print('denials:', len(d.get('permission_denials', [])))
print('is_error:', d.get('is_error'))
print('duration_ms:', d.get('duration_ms'))
"
```

各 case のレイテンシは ~23-27 秒（Claude API 往復が支配的、MCP tool
自体は sub-second）。5 case 全体で約 2 分。

## fastmcp first-launch race

Case 1 の最初の `claude mcp list` が `✗ Failed to connect` で返る
ことがある（fastmcp 初期化と health check の race）。10 秒待って
再実行すると `✓ Connected` になる。retry 1 回で OK の処理を入れるか、
登録直後は `sleep 10` を挟むのが安全。

## 標準 prompt テンプレート（mesh-mem 例）

LLM に **MCP ツールを使え** と明示することで、自己回答を避けて tool
use に誘導する：

```
Case 2 (save):
"mesh-mem MCP ツールを使って次のメモを保存してください: 「<content>」 (project=<TEST_PROJECT>)"

Case 3 (search):
"mesh-mem MCP の search_memory ツールで project=<TEST_PROJECT> のメモを 5 件まで検索して。"

Case 4 (status):
"mesh-mem MCP の get_memory_status ツールで現在のメモリ状態を見せてください。"

Case 5 (delete):
"mesh-mem MCP の delete_memory ツールで observation_id <Case 2 で取れた 32 char ID> を削除してください。reason は MCP smoke test cleanup。"
```

ID は **必ず 32 char 完全形で渡す**こと。短縮形だと LLM が省略する
リスクがあり、`delete_memory` はバリデーションで弾かれる。

## 結果検証ルール（PASS 判定）

各 case で以下を全部満たせば PASS：

1. `is_error == False`
2. `len(permission_denials) == 0`
3. tool が実際に呼ばれた痕跡が `result` に含まれる（observation_id / 件数 / 削除完了メッセージ等）
4. external verify が想定通り（Case 5 の post-delete search が 0 件 など）

どれか 1 つでも欠けたら FAIL として記録。

## 出力（呼び出し元へ報告）

`docs/poc-reports/raw/TASK-NN-mcp-smoke-result.yaml` 雛形：

```yaml
task_id: TASK-NN
issue: <ISSUE_NUM>
date: YYYY-MM-DD
verdict: PASS / FAIL / PARTIAL
mcp_server: <MCP_NAME>
mcp_bin: <MCP_BIN>
test_project: <TEST_PROJECT>
cases:
  case_1_registration:
    tool: claude_mcp_list
    result: PASS / FAIL
    evidence: "<output snippet>"
  case_2_save:
    tool: <save tool name>
    result: PASS / FAIL
    duration_ms: <N>
    permission_denials: 0
    observation_id: <32-char hex>
  case_3_search: { ... }
  case_4_status: { ... }
  case_5_delete: { ... }
incidental_findings:
  - { finding: "...", severity: "...", detail: "..." }
caveats:
  - "bypassPermissions used; UI permission flow not tested"
  - "..."
summary: |
  ...
```

## トラブルシュート

| 症状 | 原因 | 対処 |
|------|------|------|
| 全 case で permission_denials > 0 | `--permission-mode bypassPermissions` 忘れ | flag を追加 |
| Case 1 が Failed to connect | fastmcp 初期化 race | sleep 10 + retry |
| Case 5 で delete が 32-char エラー | LLM が ID を短縮した | prompt に「completely 32 文字」を明示、Case 3 の出力を再表示してから依頼 |
| LLM が tool を呼ばず自己回答する | prompt の MCP ツール明示が弱い | "MCP ツールを使って" を文頭に強調、`claude_args: --model claude-opus-4-7` で能力ブースト |
| duration が極端に長い (>60s) | Claude API 高負荷 / fallback | `--fallback-model` 設定検討 |

## 派生用途

- **複数 MCP server の同時 smoke**：MCP_NAME を変えるだけで使い回せる
- **Cron 化**：日次で smoke 走らせて Slack 通知
- **CI 統合**：`gh workflow` で merge 前に MCP server health check
