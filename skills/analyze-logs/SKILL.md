---
name: analyze-logs
description: ROSログ、kachaka-apiログを解析するスキル
---

# ログ解析スキル

ROSログおよびkachaka-apiログを解析し、エラーの原因特定を支援します。

## 使用方法

```
/analyze-logs <ログファイルパス or URL> [オプション]
```

### オプション
- `--time <HH:MM>` - 指定時刻周辺を検索（JST）
- `--range <HH:MM-HH:MM>` - 時刻範囲を指定（JST）
- `--error` - エラーのみ抽出
- `--pattern <パターン>` - 特定パターンを検索

## 解析手順

### 1. タイムゾーン変換

ログファイルのタイムスタンプ形式を確認し、必要に応じて変換してください。

| 形式 | 例 | 説明 |
|------|-----|------|
| JST | 2026-01-30 15:34:00 | 日本標準時（UTC+9） |
| UTC | 2026-01-30 06:34:00 | 協定世界時 |
| Unix Time | 1738220040 | エポック秒 |

**変換式:**
- JST → UTC: JST時刻から9時間を引く
- UTC → Unix Time: `date -d "2026-01-30 06:34:00 UTC" +%s`
- Unix Time → JST: `date -d @1738220040 '+%Y-%m-%d %H:%M:%S'`

**Pythonでの変換:**
```python
from datetime import datetime, timezone, timedelta
JST = timezone(timedelta(hours=9))

# Unix Time → JST
dt = datetime.fromtimestamp(1738220040, tz=JST)

# JST文字列 → Unix Time
dt = datetime.strptime("2026-01-30 15:34:00", "%Y-%m-%d %H:%M:%S")
dt = dt.replace(tzinfo=JST)
unix_time = int(dt.timestamp())
```

### 2. Kachaka APIエラーコード

| コード | 名前 | 説明 | 対処法 |
|--------|------|------|--------|
| 10001 | Interrupted | コマンドがキャンセルされた | 前のコマンドが完了前に新しいコマンドが送信された可能性。ログで前後のコマンドを確認 |
| 10105 | PauseButtonPressed | ポーズボタンが押された | ロボットのポーズボタンを解除 |
| 10107 | Paused | 一時停止中 | ロボットの一時停止を解除 |
| 10102 | EmergencyStop | 緊急停止 | 緊急停止を解除 |
| 10103 | ObstacleDetected | 障害物検知 | 障害物を除去 |

### 3. 重要なログパターン

#### Map mismatch
```
Map name mismatch: requested=<要求マップ>, current=<現在マップ>
```
**原因:** RMFが認識しているフロアとロボットの実際のフロアが異なる
**対処:** EV移動後に発生する場合、kachaka-api側でエラーを返すように修正が必要

#### Command not completed
```
Command is not completed for robot: <ロボット名>
```
**原因:** 前のコマンドの完了通知がfleet_adapterに届いていない
**対処:** Zenoh通信状態を確認

#### Navigation failure
```
error_code: 10001
```
**原因:** ナビゲーションが中断された
**対処:** 座標が正しいか確認（別フロアの座標を送信していないか）

### 4. ログファイルの場所

| ログ種別 | 一般的なパス | 内容 |
|----------|--------------|------|
| kachaka-api | `/home/gisen/log/<日付>/kachaka_api*.log` | Kachaka APIの通信ログ |
| RMF launch | `/home/gisen/log/<日付>/rmf_launch*.log` | RMFシステムログ |
| fleet_adapter | `ros2 topic echo /...` | リアルタイム監視 |

### 5. 解析のコツ

1. **時刻を特定する**: 問題発生時刻をJSTで確認し、ログのタイムスタンプ形式に合わせて変換
2. **前後のコンテキストを確認**: エラー発生の数秒〜数十秒前のログが重要
3. **Map mismatchを探す**: EV移動後の問題はMap mismatchが原因であることが多い
4. **コマンドIDを追跡**: 同じコマンドIDの開始・完了を追う

## 解析例

```bash
# 特定時刻のログを検索（JST 15:34 = Unix 1738220040付近）
grep -E "173822004[0-9]" /path/to/log

# Map mismatchを検索
grep "Map name mismatch" /path/to/log

# エラーコード10001を検索
grep "error_code.*10001\|errorCode.*10001" /path/to/log
```

## 参考資料

- エラーコード詳細: `/home/gisen/work/kachaka-api-for-openrmf/error_code.txt`
- kachaka-api実装: `/home/gisen/work/kachaka-api-for-openrmf/scripts/connect_openrmf_by_zenoh.py`
