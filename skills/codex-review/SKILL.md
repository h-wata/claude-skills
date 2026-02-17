---
name: codex-review
description: Codex CLIを使ってコードレビューを依頼する。未コミットの変更、ブランチ差分、特定コミットのレビューが可能。
disable-model-invocation: true
---

# Codex Review Skill

Codex CLIを使って外部AIにコードレビューを依頼します。

## 使い方

### 未コミットの変更をレビュー
```
/codex-review
```

### カスタム指示付きでレビュー
```
/codex-review セキュリティの観点でチェックして
```

### ブランチ差分をレビュー
```
/codex-review --base main
```

### 特定コミットをレビュー
```
/codex-review --commit abc1234
```

## 実行手順

1. 引数を解析して適切なcodexコマンドを構築
2. コマンドを実行してレビュー結果を取得
3. 結果をユーザーに表示

## 引数パターン

- `$ARGUMENTS` が空 → `codex review --uncommitted`
- `$ARGUMENTS` に `--base <branch>` → `codex review --base <branch>`
- `$ARGUMENTS` に `--commit <sha>` → `codex review --commit <sha>`
- それ以外 → `codex review --uncommitted "$ARGUMENTS"`

## コマンド実行

```bash
# 基本形（未コミット変更）
codex review --uncommitted

# カスタム指示付き
codex review --uncommitted "セキュリティの観点でチェック"

# ブランチ差分
codex review --base main

# 特定コミット
codex review --commit <sha>
```

## 注意

- Codex CLIが認証済みであること（`codex login`）
- 作業ディレクトリがgitリポジトリであること
