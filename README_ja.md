# Claude Skills コレクション

[Anthropic の Claude Code](https://claude.com/claude-code) のための独自スキルコレクションです。Claude の能力を特化したワークフローで拡張します。

## 概要

このリポジトリは [Anthropic Skills フレームワーク](https://github.com/anthropics/skills) を使用して構築された、本番環境で使用可能なスキルを含んでいます。各スキルは、プログレッシブディスクロージャを通じて Claude にドメイン固有の知識とワークフローを提供する、独立したパッケージです。

## 利用可能なスキル

### obsidian-work-logger

会話セッション後、Obsidian 互換のマークダウン形式で構造化された作業ログを自動作成します。

**ユースケース:**
- 開発セッションの記録
- プロジェクト作業ログの作成
- ファイル変更とリソースを含む会話サマリーの保存

**主な機能:**
- 会話内容に基づいて説明的なタイトルを自動生成
- セッション中に作成・編集されたファイルを記録
- 参照されたリソースや URL を記録
- タグ付きの Obsidian 互換フロントマターを作成
- 次のステップとフォローアップタスクを提案

### ros2-inspector

`rclpy` を使用して ROS2 システムを体系的に調査する Python スクリプトを生成します。

**ユースケース:**
- ROS2 のトピック、ノード、パラメータの調査
- 通信問題のデバッグ（QoS 互換性、メッセージ頻度）
- システムドキュメントとスナップショットの作成
- バージョン管理のための設定エクスポート

**主な機能:**
- 包括的なトピック分析（パブリッシャー、サブスクライバー、QoS、頻度）
- ノード情報の収集（接続、サービス、パラメータ）
- YAML エクスポート機能付きパラメータ抽出
- マークダウン形式でのフルシステム調査レポート
- 適切な検出タイムアウトで大規模な ROS2 システムに対応

## 前提条件

- [Claude Code CLI](https://claude.com/claude-code) がインストールされていること
- ros2-inspector 用: ROS2 環境（Humble、Iron、または Jazzy 推奨）

## インストール

### 方法1: 直接インストール（推奨）

1. このリポジトリをクローン:
```bash
git clone https://github.com/h-wata/claude-skills.git
cd claude-skills
```

2. skills リポジトリのツールを使ってスキルをインストール:
```bash
git clone https://github.com/anthropics/skills.git
cd skills

# obsidian-work-logger をインストール
python scripts/package_skill.py ../claude-skills/obsidian-work-logger

# ros2-inspector をインストール
python scripts/package_skill.py ../claude-skills/ros2-inspector
```

### 方法2: 手動 ZIP インストール

1. 目的のスキルディレクトリをダウンロード
2. ディレクトリ構造を保持したまま ZIP ファイルを作成
3. Claude Code のスキルインストールインターフェースを使用して ZIP をアップロード

### 方法3: 開発モード

アクティブな開発のために、スキルディレクトリを Claude Code のスキルディレクトリ（プラットフォームによって場所が異なります）に直接リンクできます。

## 使用方法

### obsidian-work-logger の使用

会話セッション完了後、単純に以下のように尋ねます:

```
「このセッションの作業ログを作成して」
「今日やったことを Obsidian に記録して」
「このセッションをノートに保存して」
```

Claude は自動的に:
1. セッションに関する情報を収集
2. 説明的なタイトルを生成
3. 構造化されたマークダウンドキュメントを作成
4. Obsidian ディレクトリの場所を確認
5. 作業ログファイルを書き込み

### ros2-inspector の使用

実行中の ROS2 システムを調査するには:

```
「このシステムの ROS2 トピックを調査して」
「ROS_DOMAIN_ID=13 で実行中のノードを表示して」
「トピックの周波数と QoS 設定を分析して」
「すべてのパラメータを YAML にエクスポートして」
```

Claude は以下を実行:
1. 調査範囲を決定
2. 適切な調査スクリプトを選択してカスタマイズ
3. 適切な検出タイムアウトでスクリプトを実行
4. マークダウンまたは YAML で結果を整形
5. オプションでシステム図を作成

## 開発

### スキル構造

各スキルは Anthropic Skills 仕様に従います:

```
skill-name/
├── SKILL.md              # YAML フロントマター付きスキル指示
├── scripts/              # 実行可能スクリプト（オプション）
├── references/           # ドキュメントリソース（オプション）
└── assets/              # テンプレートと出力ファイル（オプション）
```

### 新しいスキルの作成

1. [skills リポジトリ](https://github.com/anthropics/skills) の初期化スクリプトを使用:
```bash
python scripts/init_skill.py
```

2. 生成された `SKILL.md` を編集して指示を記述

3. `scripts/`、`references/`、または `assets/` ディレクトリにサポートファイルを追加

4. 検証とパッケージング:
```bash
python scripts/package_skill.py path/to/your-skill
```

### 貢献

貢献を歓迎します！以下の手順でお願いします:

1. このリポジトリをフォーク
2. 機能ブランチを作成
3. スキルを十分にテスト
4. 以下を含むプルリクエストを送信:
   - スキルの目的の明確な説明
   - 使用例
   - 前提条件や依存関係

## 参考文献

- [Anthropic Skills Repository](https://github.com/anthropics/skills)
- [What are skills? - Claude Support](https://support.claude.com/en/articles/12512176-what-are-skills)
- [Equipping agents for the real world with Agent Skills](https://anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Obsidian Documentation](https://help.obsidian.md/)

## ライセンス

このプロジェクトは Apache License 2.0 の下でライセンスされています - 詳細は LICENSE ファイルを参照してください。

スキルは教育および開発目的でそのまま提供されます。外部システムやサービスと対話するスキルを使用する際は、適切な権限があることを確認してください。

## 謝辞

- [Anthropic Skills フレームワーク](https://github.com/anthropics/skills) を使用して構築
- ROS2 インスペクタは `rclpy` クライアントライブラリに基づく
- ワークロガーは [Obsidian](https://obsidian.md/) マークダウン互換性のために設計
