---
name: ros-analyze
description: ROS2 ログ解析コマンド
---

# ROS2 ログ解析コマンド

ROS2システムの状態を解析し、問題を検出するためのコマンドです。

## 引数

$ARGUMENTS

引数が空の場合は、対話的に解析対象を選択してください。

## 解析モード

引数に応じて以下の解析を実行してください：

### 1. `status` - システム状態確認
以下のコマンドを実行して現在のROS2システム状態を確認：
```bash
ros2 node list
ros2 topic list
ros2 service list
```
結果をまとめて報告してください。

### 2. `topics [トピック名]` - トピック解析
指定されたトピック（または主要トピック）の状態を確認：
```bash
ros2 topic info <topic_name>
ros2 topic hz <topic_name>
ros2 topic echo <topic_name> --once
```
パブリッシャー/サブスクライバーの数、周波数、最新メッセージを報告。

### 3. `errors` - エラー・警告検出
ROS2ログからエラーと警告を検出：
```bash
ros2 topic echo /rosout --once -n 50
```
ERROR、WARNING レベルのメッセージを抽出し、問題の原因と対処法を提案。

### 4. `fleet` - フリート状態確認（RMF用）
RMFフリートの状態を確認：
```bash
ros2 topic echo /fleet_states --once
ros2 topic echo /mutex_group_states --once
ros2 topic echo /task_summaries --once
```
ロボットの位置、タスク状態、Mutex Group状態を報告。

### 5. `trace <パターン>` - パターン検索
指定パターンを含むメッセージを検索。/rosout から該当するログを抽出。

### 6. `nodes [ノード名]` - ノード詳細
指定ノードの詳細情報を取得：
```bash
ros2 node info <node_name>
```
サブスクリプション、パブリッシャー、サービス、パラメータを報告。

## 出力形式

解析結果は以下の形式で報告してください：

1. **概要**: 全体的な状態のサマリー
2. **詳細**: 各項目の詳細情報
3. **問題点**: 検出された問題（あれば）
4. **推奨アクション**: 問題がある場合の対処法

## 注意事項

- コマンド実行時にタイムアウトが発生する場合は、`--once` や `-n` オプションで制限してください
- 大量の出力が予想される場合は、最新のメッセージのみを取得してください
- 日本語で結果を報告してください
