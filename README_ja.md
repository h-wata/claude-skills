# Claude Skills コレクション

[Claude Code](https://docs.anthropic.com/en/docs/claude-code) のための独自スキルコレクションです。特化したワークフローで Claude の能力を拡張します。

## 概要

このリポジトリでは、個人用の Claude Code スキルを一元管理しています。各スキルは `SKILL.md` を含む独立したディレクトリで、Claude にドメイン固有の指示とワークフローを提供します。

```
~/work/claude-skills/              <- このリポジトリ
├── .claude-plugin/
│   └── marketplace.json           <- プラグインマーケットプレイス定義
├── config.json                    <- 共有設定
├── skills/
│   ├── memo/SKILL.md
│   ├── survey/SKILL.md
│   └── ...
```

## 利用可能なスキル

### ドキュメント・ログ

| スキル | 説明 | コマンド |
|--------|------|----------|
| **memo** | Obsidian Daily ノートへのクイックメモ | `/memo` |
| **work-log** | 現在の会話から作業ログを記録 | `/work-log` |
| **obsidian-work-logger** | セッションの構造化された作業ログを Obsidian 形式で出力 | `/obsidian-work-logger` |
| **interview** | ブレスト・振り返り・メモ作成のためのインタラクティブインタビュー | `/interview` |

### コード分析・レビュー

| スキル | 説明 | コマンド |
|--------|------|----------|
| **survey** | PDF・リポジトリ・Web ドキュメントの索引サマリーを作成 | `/survey` |
| **write-spec** | survey 結果やソースコードから実装仕様書を生成 | `/write-spec` |
| **cross-review** | 複数ドキュメント間の整合性レビュー（5 観点） | `/cross-review` |
| **git-history** | Git 履歴を調査し、変更の経緯を追跡 | `/git-history` |
| **codex-consult** | Codex CLI を使った設計相談（read-only） | `/codex-consult` |
| **codex-review** | Codex CLI を使ったコードレビュー | `/codex-review` |

### ROS2・ロボティクス

| スキル | 説明 | コマンド |
|--------|------|----------|
| **ros2-inspector** | rclpy スクリプトを生成して ROS2 システムを調査 | `/ros2-inspector` |
| **ros-analyze** | ROS2 システム状態の解析（ノード、トピック、フリート） | `/ros-analyze` |
| **analyze-logs** | ROS / kachaka-api ログ解析（エラーコードリファレンス付き） | `/analyze-logs` |

### 計画・ワークフロー

| スキル | 説明 | コマンド |
|--------|------|----------|
| **plan-with-qa** | Q&A で仕様を明確化しながら実装計画を立てる | `/plan-with-qa` |
| **meeting-minutes** | 議事録の作成・更新 | `/meeting-minutes` |

## インストール

### 方法 1: プラグインインストール

このリポジトリをプラグインマーケットプレイスとして登録し、全スキルをプラグインとしてインストール:

```bash
# マーケットプレイスとして追加
claude plugin marketplace add h-wata/claude-skills

# 全スキルをインストール
claude plugin install h-wata-skills@h-wata-claude-skills
```

または対話式のプラグインマネージャーを使用:

```
/plugin
```

### 方法 2: Symlink

開発用、または個別にスキルを管理したい場合:

```bash
git clone https://github.com/h-wata/claude-skills.git ~/work/claude-skills
cd ~/work/claude-skills

# 全スキルの symlink を作成
mkdir -p ~/.claude/skills
for skill in skills/*/; do
  skill_name="$(basename "$skill")"
  [ -f "$skill/SKILL.md" ] && ln -sf "$(pwd)/$skill" ~/.claude/skills/$skill_name
done
```

### 設定

`config.json` には複数のスキルが参照する共有設定（Obsidian の Daily ノートパスなど）を格納しています:

```json
{
    "obsidian_daily_path": "~/Documents/my-obsidian/Daily"
}
```

## リポジトリ構造

```
claude-skills/
├── .claude-plugin/
│   └── marketplace.json      # プラグインマーケットプレイス定義
├── config.json               # スキル共有設定
├── skills/
│   └── skill-name/
│       ├── SKILL.md          # YAML フロントマター付きスキル指示（必須）
│       ├── scripts/          # 実行可能スクリプト（オプション）
│       ├── references/       # ドキュメントリソース（オプション）
│       └── assets/           # テンプレートと出力ファイル（オプション）
```

### SKILL.md フロントマター

```yaml
---
name: skill-name
description: スキルの説明
allowed-tools: Read, Grep, Bash    # 許可プロンプトなしで使えるツール
disable-model-invocation: true     # /name でのみ手動呼び出し
---
```

## 前提条件

- [Claude Code](https://docs.anthropic.com/en/docs/claude-code) がインストール済み
- ROS2 スキル用: ROS2 環境（Humble、Iron、または Jazzy）

## 参考文献

- [Claude Code Skills ドキュメント](https://docs.anthropic.com/en/docs/claude-code/skills)
- [Claude Code Plugins ドキュメント](https://docs.anthropic.com/en/docs/claude-code/plugins)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Obsidian Documentation](https://help.obsidian.md/)

## ライセンス

Apache License 2.0 - 詳細は LICENSE ファイルを参照してください。
