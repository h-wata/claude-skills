---
name: codex-consult
description: Codex CLIを使って設計相談やコードについての質問をする。read-onlyモードで安全に実行。
disable-model-invocation: true
---

# Codex Consult Skill

Codex CLI を使って **外部 AI（OpenAI）に設計相談・コード質問** をするスラッシュコマンド。回答本体は必ず codex に書かせる（呼び出し側が自前で回答を書くのは禁止）。

## 絶対遵守ルール

1. `codex exec` を **1 回だけ** 実行する。失敗しても再試行せず、失敗として報告する。
2. **リポジトリを改変しない。** プロンプトにも「これは consult、ファイル編集禁止」と明示する。codex が提案としてコードを示すのは可だが、実ファイルへの書き込みは不要。
3. codex 出力が空・エラーなら、自前で回答を書かず「Codex 相談失敗」として報告する。
4. sandbox オプション（`-c sandbox_mode=danger-full-access`）を省略・変更しない。これがないと bwrap が user namespace を作れない環境で codex がファイル参照系の質問に答えられない。

## 使い方

```
/codex-consult <質問文>
```

## 実行コマンド

```bash
codex exec -c sandbox_mode=danger-full-access "read-only consultation — do not modify any files. question: $ARGUMENTS" < /dev/null 2>&1
```

- `-c sandbox_mode=danger-full-access`: bwrap 迂回（必須）
- プロンプト冒頭の `read-only consultation — do not modify any files` で codex 側に変更禁止を指示
- `< /dev/null`: stdin ハング回避
- `2>&1`: エラー出力も取得

**作業ディレクトリを指定したい場合:**

```bash
codex exec -C <dir> -c sandbox_mode=danger-full-access "read-only consultation — do not modify any files. question: $ARGUMENTS" < /dev/null 2>&1
```

## 実行手順

1. `$ARGUMENTS` を上記コマンドに埋め込んで 1 回実行。
2. タイムアウトは 120 秒程度見込む。
3. 成功判定（下記）を行い、結果を報告。

## 成功判定（機械的に）

codex 出力が **下記のいずれかを含めば失敗**:
- 空
- `unable to inspect` / `could not` / `sandbox` / `Denied` / `bwrap` のいずれか
- `Error:` で始まる行
- `Not inside a trusted directory` （codex の信頼ディレクトリ未登録）

該当しなければ成功。

## 報告フォーマット

**成功時:**

```
## Codex 相談結果

### 質問
[ユーザー質問をそのまま]

### Codex の回答
[codex 出力の本文]

### 推奨アクション
- [codex が提示した次のステップがあれば抽出]
```

**失敗時:**

```
## Codex 相談失敗

### 原因
[codex 出力の該当部分を短く引用]

### 対処案
- `~/.codex/config.toml` で sandbox_mode / trusted projects を確認
- 未認証なら `codex login`
- 信頼ディレクトリ外なら該当パスを projects に追加、もしくは `-C <trusted-dir>` を付ける
```

**絶対にやってはいけない:** 失敗時に自分が知識で答えて埋め合わせること。

## 注意

- 長い質問は引用符で囲む
- 作業ディレクトリが codex の trusted projects に登録されていないと弾かれることがある
