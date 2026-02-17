---
name: codex-consult
description: Codex CLIを使って設計相談やコードについての質問をする。read-onlyモードで安全に実行。
disable-model-invocation: true
---

# Codex Consult Skill

Codex CLIを使って外部AIに設計相談やコードについて質問します。

## 使い方

### コードについて相談
```
/codex-consult この設計でエレベーター乗降シーケンスを実装するのは適切？
```

### 特定ファイルについて質問
```
/codex-consult robot_client_api.pyのget_command_is_completedメソッドのロジックを説明して
```

### アーキテクチャ相談
```
/codex-consult ROS2とZenohの連携で、Pub/Subの信頼性を高める方法は？
```

## 実行手順

1. ユーザーの質問を受け取る
2. `codex exec` をread-onlyモードで実行
3. 結果を表示

## コマンド

```bash
codex exec --sandbox read-only "$ARGUMENTS"
```

## オプション

- `--sandbox read-only`: ファイル変更なし、読み取りのみ
- 必要に応じて `-C <dir>` で作業ディレクトリ指定

## 注意

- read-onlyモードなのでファイルは変更されません
- 長い質問は引用符で囲んでください
