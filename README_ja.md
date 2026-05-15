# Claude Skills コレクション

[Claude Code](https://docs.anthropic.com/en/docs/claude-code) のための独自スキル集です。特化したワークフローで Claude の能力を拡張します。

[English README](README.md)

## 概要

このリポジトリは個人用 Claude Code スキルを一元管理します。各スキルは `skills/` 配下に独立したディレクトリを持ち、`SKILL.md` でドメイン固有の指示・ワークフローを Claude に提供します。

```
~/work/claude-skills/              <- このリポジトリ
├── .claude-plugin/
│   └── marketplace.json           <- プラグインマーケットプレイス定義
├── config.json                    <- 共有設定（Obsidian パス等）
├── skills/
│   ├── memo/SKILL.md
│   ├── survey/SKILL.md
│   └── ...
```

## 収録スキル（全 26 種）

### ドキュメント・ログ

| スキル | 説明 | コマンド |
|--------|------|----------|
| **memo** | Obsidian Daily ノートへのクイックメモ | `/memo` |
| **work-log** | 現在の会話から作業ノートを記録 | `/work-log` |
| **obsidian-work-logger** | セッションの構造化作業ログを Obsidian 形式で出力 | `/obsidian-work-logger` |
| **interview** | 考えや意見を引き出すインタビュー形式のメモ作成 | `/interview` |
| **meeting-minutes** | 議事録の作成・更新 | `/meeting-minutes` |
| **survey** | PDF・リポジトリ・Web ドキュメントの索引サマリーを作成 | `/survey` |

### 計画・仕様化

| スキル | 説明 | コマンド |
|--------|------|----------|
| **plan-with-qa** | AskUserQuestion で曖昧な仕様を潰しながら Markdown 実装計画を出力 | `/plan-with-qa` |
| **write-spec** | survey 結果やソースコード・PDF 定義をもとに仕様書を生成 | `/write-spec` |
| **empirical-prompt-tuning** | バイアスを排した実行者と両面評価でエージェント向け指示を反復改善 | `/empirical-prompt-tuning` |

### レビュー・分析

| スキル | 説明 | コマンド |
|--------|------|----------|
| **cross-review** | 複数ドキュメント間の整合性レビュー | `/cross-review` |
| **codex-consult** | Codex CLI を使った設計相談（read-only） | `/codex-consult` |
| **codex-review** | Codex CLI を使ったコードレビュー | `/codex-review` |
| **config-analyzer** | Claude Code 設定（CLAUDE.md / settings / hooks / skills / MCP）をベストプラクティスと突き合わせて監査 | `/config-analyzer` |

### Git・リリースワークフロー

| スキル | 説明 | コマンド |
|--------|------|----------|
| **git-history** | Git 履歴を調査し、変更の経緯を追跡 | `/git-history` |
| **safe-pathspec-commit** | 並行 worker の未コミット WIP を巻き込まずに対象ファイルだけ commit | `/safe-pathspec-commit` |
| **inherit-wip** | 中断 / 他 worker の未コミット WIP を引き継いで完了させる | `/inherit-wip` |
| **release-apply** | drafts のリリースノート / CHANGELOG / migration を反映し tag・GitHub Release・Issue close まで実行 | `/release-apply` |

### アーキテクチャ・意思決定

| スキル | 説明 | コマンド |
|--------|------|----------|
| **adr** | Architecture Decision Records の作成 / 一覧 / supersede（手動呼び出し専用） | `/adr` |

### ROS2・ロボティクス・3DGS

| スキル | 説明 | コマンド |
|--------|------|----------|
| **ros2-inspector** | rclpy スクリプトを生成して ROS2 のトピック・ノード・パラメータを調査 | `/ros2-inspector` |
| **ros-analyze** | ROS2 システム状態の解析（ノード・トピック・フリート） | `/ros-analyze` |
| **ros2-launch-debug** | ROS2 launch ファイルのデバッグ支援（パラメータ・依存・QoS） | `/ros2-launch-debug` |
| **analyze-logs** | ROS / kachaka-api ログ解析（エラーコードリファレンス付き） | `/analyze-logs` |
| **nerfstudio-trainer** | nerfstudio splatfacto（3D Gaussian Splatting）学習をプロジェクト調整済みパラメータで起動 | `/nerfstudio-trainer` |

### mesh-mem / MCP / 分散ツール

| スキル | 説明 | コマンド |
|--------|------|----------|
| **add-mesh-peer** | 既存 mesh-mem (Zenoh) メッシュに新ホストを peer として参加させる一連の手順 | `/add-mesh-peer` |
| **mcp-smoke-via-claude-p** | `claude -p --output-format json` で MCP server を 5-case 自動 smoke | `/mcp-smoke-via-claude-p` |

### セットアップ・設定

| スキル | 説明 | コマンド |
|--------|------|----------|
| **setup-claude-local** | ローカル ADR + `.claude/CLAUDE.md` + Obsidian vault 連携をセットアップ | `/setup-claude-local` |
| **config-analyzer** | （上記「レビュー・分析」参照） | `/config-analyzer` |

## インストール

### 方法 1: プラグインインストール（推奨）

このリポジトリをプラグインマーケットプレイスとして登録し、収録スキルを一括導入:

```bash
# マーケットプレイスとして追加
claude plugin marketplace add h-wata/claude-skills

# 全スキルをインストール
claude plugin install h-wata-skills@h-wata-claude-skills
```

または対話式プラグインマネージャーから:

```
/plugin
```

### 方法 2: Symlink（開発用）

```bash
git clone https://github.com/h-wata/claude-skills.git ~/work/claude-skills
cd ~/work/claude-skills

mkdir -p ~/.claude/skills
for skill in skills/*/; do
  skill_name="$(basename "$skill")"
  [ -f "$skill/SKILL.md" ] && ln -sf "$(pwd)/$skill" ~/.claude/skills/$skill_name
done
```

### 設定

`config.json` は複数スキルが参照する共有設定（Obsidian Daily ノートパスなど）を保持します:

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
│   └── <skill-name>/
│       ├── SKILL.md          # YAML frontmatter + 指示（必須）
│       ├── scripts/          # 実行ヘルパー（任意）
│       ├── references/       # 参考資料（任意）
│       └── assets/           # テンプレート / 出力（任意）
```

### SKILL.md フロントマター

```yaml
---
name: skill-name
description: スキルの説明・利用タイミング
allowed-tools: Read, Grep, Bash    # 許可プロンプトなしで使えるツール
disable-model-invocation: true     # /name でのみ手動呼び出し
---
```

## 前提条件

- [Claude Code](https://docs.anthropic.com/en/docs/claude-code) がインストール済み
- ROS2 系スキル: ROS2 環境（Humble / Iron / Jazzy）
- `nerfstudio-trainer`: nerfstudio と CUDA GPU
- `add-mesh-peer` / `mcp-smoke-via-claude-p`: Zenoh / MCP 環境
- Obsidian 連携スキル: Obsidian vault と vault を指す `config.json`

## 参考文献

- [Claude Code Skills ドキュメント](https://docs.anthropic.com/en/docs/claude-code/skills)
- [Claude Code Plugins ドキュメント](https://docs.anthropic.com/en/docs/claude-code/plugins)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Obsidian Documentation](https://help.obsidian.md/)

## ライセンス

Apache License 2.0 — 詳細は [LICENSE](LICENSE) を参照してください。
