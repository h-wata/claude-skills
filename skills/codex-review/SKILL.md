---
name: codex-review
description: Codex CLIを使ってコードレビューを依頼する。未コミットの変更、ブランチ差分、特定コミットのレビューが可能。
disable-model-invocation: true
---

# Codex Review Skill

Codex CLI を使って **外部 AI（OpenAI）にコードレビューを依頼** するスラッシュコマンド。レビュー本体は必ず codex に書かせる（呼び出し側が diff を読んで自前でレビューを書くのは禁止）。

## 絶対遵守ルール

1. 下記の引数パターンで決まる codex review コマンドを **1 回だけ** 実行する。失敗しても別コマンド（`--base` / `--commit` への切替など）に fallback しない。
2. **リポジトリを改変しない** (`git add` / `git commit` / `git stash` 等の禁止)。`git status` / `git diff` 等の読み取り系のみ許可。
3. codex 出力が失敗（後述の判定基準）なら、自前でレビューを書かず「Codex レビュー失敗」として失敗フォーマットで報告する。
4. sandbox オプション（`-c sandbox_mode=danger-full-access`）を省略・変更しない。これがないと bwrap が user namespace を作れない環境で codex が diff を読めず空出力になる。

## 使い方

`codex review` は `--uncommitted` / `--base` / `--commit` の各 scope フラグと自由文 PROMPT を **排他** で扱う。カスタム指示を渡す場合は scope フラグを外し、scope を PROMPT 本文に含めて指示する。

| コマンド | 呼び出す codex |
|---|---|
| `/codex-review` | `codex review --uncommitted -c sandbox_mode=danger-full-access 2>&1` |
| `/codex-review <自由文>` | `codex review -c sandbox_mode=danger-full-access "未コミットの変更をレビュー対象とする。追加指示: <自由文>" 2>&1` |
| `/codex-review --base <branch>` | `codex review --base <branch> -c sandbox_mode=danger-full-access 2>&1` |
| `/codex-review --commit <sha>` | `codex review --commit <sha> -c sandbox_mode=danger-full-access 2>&1` |

### 引数パターン判定

- `$ARGUMENTS` が空 → uncommitted (scope フラグ形)
- `$ARGUMENTS` が `--base <branch>` で始まる → base 差分
- `$ARGUMENTS` が `--commit <sha>` で始まる → commit
- それ以外 → uncommitted + カスタム指示（scope フラグを付けず、scope と追加指示を PROMPT 本文に含める）

## 実行手順

1. `git status --short` で変更状況を確認。uncommitted パターンで差分ゼロなら codex を呼ばずに「レビュー対象なし」フォーマット（後述）で返す。
2. 上記の表から 1 コマンドを選んで実行。タイムアウトは 180 秒程度見込む。
3. 成功判定（下記）を行い、成功/失敗フォーマットで報告。

## 成功判定（機械的に）

codex 出力が **下記のいずれかを含めば失敗**:
- 空または codex 本体応答（ヘッダ行以降、`codex\n` ラベル以下の本文）が 1 行のみ
- `unable to inspect` / `could not inspect` のいずれかの文字列
- `Sandbox(Denied` / `bwrap:` のいずれかの文字列（bwrap 迂回失敗の実エラー）
- `Error:` で始まる行

**注意:** codex は正常時も `sandbox: <mode>` をヘッダに出力するため、裸の `sandbox` 文字列を判定に使ってはいけない。上記の具体パターンのみを使うこと。

上記に該当しなければ成功。

## 報告フォーマット

**レビュー対象なし時（手順 1 で短絡した場合）:**

```
## レビュー対象なし

作業ディレクトリに未コミット変更がないため、Codex は呼び出していません。
変更を加えてから再実行するか、`--base <branch>` / `--commit <sha>` で対象を指定してください。
```

**成功時:**

```
## Codex レビュー結果

### 指摘事項
- [codex 出力からそのまま抽出]

### 推奨事項
- [codex 出力からそのまま抽出]

### 評価
[codex 出力の総評部分]
```

**codex 出力がこの 3 節構造と合わない場合の扱い:**
- codex が `Review comment:` 形式などの単一段落で返すことがある。その場合は **codex 出力の本文を「指摘事項」に丸ごと原文のまま貼る** こと（勝手に日本語訳・要約・配分しない）。
- 「推奨事項」「評価」に該当する内容が codex 出力に無ければ、そのセクションは `- なし` と書くか、セクションごと省略する。
- codex の出力構造を **改造してはいけない**。「推奨事項」が空だからといって「指摘の裏返し」を勝手に書き起こすのは自前補完の一種で禁止。
- codex が同一文を 2 回以上連続で繰り返した場合（codex 側の既知の挙動）は 1 回にまとめてよい（重複除去のみ許可、内容は一字一句変えない）。

**失敗時:**

```
## Codex レビュー失敗

### 原因
[codex 出力の該当部分を短く引用]

### 対処案
- `~/.codex/config.toml` で sandbox_mode を確認
- bwrap が user namespace を作れない環境では `-c sandbox_mode=danger-full-access` 必須
- 未認証の可能性があれば `codex login` を案内
```

**絶対にやってはいけない:** 失敗時に自分で diff を読み直して「追加で気づいた点」を書くこと。

## 注意

- Codex CLI が認証済みであること（未認証なら `codex login`）
- 作業ディレクトリが git リポジトリであること
