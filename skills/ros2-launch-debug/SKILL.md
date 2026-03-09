---
name: ros2-launch-debug
description: ROS2 launchファイルのデバッグ支援。パラメータ解析、ノード依存関係の追跡、QoS不一致検出、トピック接続の検証を実施する。「launchが動かない」「パラメータが反映されない」「ノードが起動しない」等の問題調査に使用。
---

# ROS2 Launch Debug

## Overview

ROS2 launchファイルの構造解析・パラメータ検証・依存関係追跡を行い、起動時の問題を特定するスキル。

## When to Use

- "launchが動かない" / "ノードが起動しない"
- "パラメータが反映されない"
- "launchファイルを調査して"
- "QoSが合わない"
- "ノードの依存関係を確認"
- launch.pyの構造理解や修正が必要な場面全般

## How to Use

### Step 1: Launch ファイルの特定と読込

ユーザーの指定するlaunchファイル、または作業ディレクトリ内のlaunchファイルを特定する。

```bash
find . -name "*.launch.py" -o -name "*.launch.xml" -o -name "*.launch.yaml" | head -20
```

対象launchファイルを読み込み、以下を抽出する:

- `Node()` / `LifecycleNode()` の定義一覧
- `LaunchConfiguration` / `DeclareLaunchArgument` の一覧
- `IncludeLaunchDescription` による子launch参照
- `ComposableNodeContainer` / `LoadComposableNode` の使用
- `GroupAction`, `OpaqueFunction` 等の動的構成
- `remappings` の設定

### Step 2: パラメータチェーン解析

パラメータの流れを追跡する:

1. **宣言チェック**: `DeclareLaunchArgument` のデフォルト値と型
2. **参照チェック**: `LaunchConfiguration` が正しい名前を参照しているか
3. **YAML読込チェック**: `parameters=[yaml_file]` のパスが存在するか
4. **型変換チェック**: 数値パラメータに `PythonExpression` での型変換が必要か
5. **上書きチェック**: 同じパラメータが複数箇所で上書きされていないか

```python
# よくあるミス例
Node(
    parameters=[{
        'robot_name': LaunchConfiguration('robot_name'),  # str型で渡される
        'max_speed': LaunchConfiguration('max_speed'),     # これもstr → floatに変換必要
    }]
)
```

### Step 3: ノード依存関係の解析

起動順序の問題を検出:

1. **TF依存**: `robot_state_publisher` より前にTFを要求するノードがないか
2. **Service依存**: `lifecycle_manager` が管理するノードの起動順序
3. **Topic依存**: サブスクライバがパブリッシャより先に起動してメッセージを受け取れない場合
4. **名前空間衝突**: 同じノード名やトピック名が競合していないか

### Step 4: QoS互換性チェック

launchファイル内およびノードのソースコード（見つかれば）からQoS設定を確認:

| 設定 | 互換条件 |
|------|----------|
| Reliability | Publisher=BestEffort, Subscriber=Reliable → NG |
| Durability | Publisher=Volatile, Subscriber=TransientLocal → NG |
| History | 深刻な問題になることは少ないが、Depth差に注意 |

### Step 5: 実行時検証（ROS2システム稼働中の場合）

ROS2が動いている場合は追加で以下を実行:

```bash
# ノードの起動確認
ros2 node list

# 期待されるトピックが存在するか
ros2 topic list

# 特定ノードの状態確認
ros2 lifecycle get <node_name>

# パラメータの実際の値
ros2 param dump <node_name>
```

### Step 6: 報告

日本語で以下のフォーマットで報告:

```markdown
## Launch解析結果

### 構成概要
- **Launchファイル**: path/to/file.launch.py
- **定義ノード数**: N
- **子Launch数**: N
- **LaunchArgument数**: N

### ノード一覧
| ノード名 | パッケージ | 実行ファイル | 名前空間 |
|----------|-----------|------------|---------|
| ... | ... | ... | ... |

### 検出された問題

#### 重大
1. **[問題の概要]**
   - 該当箇所: `ファイル:行番号`
   - 原因: ...
   - 修正案: ...

#### 警告
1. **[警告の概要]**
   - 影響: ...
   - 推奨: ...

### パラメータフロー
LaunchArgument → Node Parameter の流れを可視化

### 推奨アクション
1. ...
2. ...
```

## Common Issues Checklist

- [ ] `FindPackageShare` のパッケージがインストールされているか
- [ ] YAML設定ファイルのパスが正しいか
- [ ] `PythonExpression` で数値型に変換しているか
- [ ] `condition=IfCondition()` の条件が正しいか
- [ ] `remappings` のトピック名にtypoがないか
- [ ] `namespace` の二重適用（GroupAction + Node）がないか
- [ ] `use_sim_time` が一貫して設定されているか
- [ ] composable node の `target_container` が正しいコンテナを指しているか

## ROS2 Launch Python API Quick Reference

| クラス | 用途 |
|--------|------|
| `Node` | 通常のノード起動 |
| `LifecycleNode` | ライフサイクルノード起動 |
| `ComposableNodeContainer` | コンポーネントコンテナ |
| `LoadComposableNode` | コンポーネント動的ロード |
| `IncludeLaunchDescription` | 子launch読込 |
| `GroupAction` | 名前空間・条件付きグループ |
| `DeclareLaunchArgument` | 引数宣言 |
| `LaunchConfiguration` | 引数参照 |
| `OpaqueFunction` | 動的構成（Python関数） |
| `SetEnvironmentVariable` | 環境変数設定 |
| `ExecuteProcess` | 任意プロセス実行 |
