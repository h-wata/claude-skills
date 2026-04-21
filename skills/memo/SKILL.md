---
name: memo
description: Quick memo to Obsidian Daily note
allowed-tools: Bash, Read
---

First, read ~/work/claude-skills/config.json to get the `obsidian_daily_path`.

Then append the following memo to {obsidian_daily_path}/$(date +%Y-%m-%d).md

## Steps
1. Read config.json to get output path
2. Create directory and file if they don't exist (`mkdir -p` + `touch`)
3. Add timestamp heading (HH:MM format, obtained from `date +%H:%M` in Bash at execution time)
4. Append to end of file

## Append rules

### 時刻の事前解決（重要）
quoted heredoc (`<<'EOF'`) はシェル展開を**全て抑止する**ので、本文中に `$(date +%H:%M)` や `$TIME` を書いても展開されない。時刻は先に Bash で `date +%H:%M` を実行して結果文字列を取得し、heredoc 本文には**その結果をリテラルで埋め込む**こと。以下の2段階で実装する:

1. Bash で `date +%H:%M` を実行（例: 結果が `18:42`）
2. 次に書き込むコマンドの heredoc 本文に `18:42` を直接タイプする（変数ではなくリテラルとして）

### エスケープ
`$ARGUMENTS` はバッククォート・`$`・引用符などを含みうる。必ず **quoted heredoc** で書き込むこと。`echo` や unquoted heredoc はシェル展開されて内容が壊れる。

### 書き込みコマンド例
時刻取得結果が `18:42`、`$ARGUMENTS` が `明日MTGをレビュー` の場合:

```bash
FILE=/path/to/2026-04-21.md
# 既存ファイルが有る場合のみ、既存最終行と新見出しの間を空行1行で区切る
# - ファイルが空なら何もしない（新規書き込みで冒頭に空行を作らない）
# - 末尾が LF 終端でないなら LF を 1 つ足して段落区切りを確保
if [ -s "$FILE" ]; then
  [ "$(tail -c1 "$FILE" | wc -l)" -eq 0 ] && printf '\n' >> "$FILE"
  printf '\n' >> "$FILE"
fi
cat >> "$FILE" <<'EOF'
## 18:42

明日MTGをレビュー
EOF
```

（`## 18:42` と本文は **実行時にリテラルで差し替える**。変数は使わない。heredoc 冒頭に空行は入れない — 既存ファイルがある場合の空行は上の `if` ブロックで先に担保する）

### その他
- **既存ファイルへの追記**: 既存最終行と新 `## HH:MM` 見出しの間に空行 1 行を入れる（上の `if` ブロックで、LF 終端担保 + 空行 1 行の 2 段階で処理）
- **新規ファイル**: 冒頭にファイルレベル見出し（`# 2026-04-21` など）は付けない。日付はファイル名に入っている。冒頭空行も作らない（上の `if` ブロックが空ファイルをスキップするため自動的に担保される）
- **allowed-tools は `Bash, Read`**: Write ツールは使わない。追記は上の quoted heredoc で

## Memo content
$ARGUMENTS
