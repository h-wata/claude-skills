---
name: add-mesh-peer
description: 既存 mesh-mem (Zenoh) メッシュに新しいホストを peer として参加させる。zenoh apt インストール → venv → config 生成 → 既存 peer の listen 拡張 → 起動 → 双方向 replication 確認 → 任意で Claude Code MCP 登録、までを一気通貫。
allowed-tools: Bash, Read, Edit, Write, Glob, Grep
---

# add-mesh-peer

mesh-mem の N-host メッシュに 1 ホスト追加するときの定番手順。WSL2 への 3rd peer 追加（2026-05-04）で確立したパターン。

## 前提条件

- 既存メッシュは少なくとも 1 router 稼働中（典型的には Home / Office）
- 新規 host にネットワーク疎通あり（少なくとも既存 router の listen IP のいずれか 1 つに TCP/7447 が届く）
- 新規 host で `sudo`、`python3 >= 3.10`、`apt-get` が使える
- 新規 host から既存 router へ：ICMP / TCP/7447 の outbound が通ること（先に `ping` / `nc -zv` で確認）

## いつ使うか

- 個人利用の N-device mesh を 1 台ずつ拡張するとき
- 既存 mesh と隔離したテスト peer を追加するとき
- WSL2 / 別ホスト / VPS を mesh に組み込むとき

## 入力（必ず確認すること）

| 項目 | 例 | 用途 |
|------|------|------|
| `NEW_PEER_IP` | 192.168.3.29 | 新規 host の LAN IP（他 peer がダイヤルする） |
| `NEW_PEER_HOSTNAME` | wsl-desktop | log や naming で識別 |
| `EXISTING_PEER_IP` | 192.168.3.12 | 新規 host が outbound で connect する既存 peer。多段ホップでも 1 個経由できれば transit で全 mesh に届く |
| `MESH_MEM_AGENT_FAMILY` | claude | 識別子。既存と統一 |
| `MESH_MEM_CLIENT_ID` | claude-code-wsl | 既存と区別できる名前（`-wsl` `-mac` 等のサフィックス推奨） |

## ステップ

### 1. 新規 host の初期 survey

```bash
ssh <new_host>
uname -a
cat /etc/os-release | head -3
python3 --version
which zenohd && zenohd --version || echo "zenohd absent"
ls -la ~/work/mesh-mem 2>&1 | head -3   # 既に clone 済か確認
grep -qi microsoft /proc/version && echo WSL2 || echo native
```

確認ポイント：
- Ubuntu / Debian 系であること（apt が使える）
- python3 >= 3.10
- mesh-mem clone が無ければ clone（`git clone https://github.com/h-wata/mesh-mem ~/work/mesh-mem`）

### 2. zenoh apt repo + zenohd インストール

既存 host から GPG key を scp で持ってくるのが一番確実：

```bash
# dispatcher 側で
scp -P <ssh_port> /etc/apt/keyrings/zenoh-public-key.gpg <user>@<new_host>:/tmp/
```

新規 host で：

```bash
sudo bash -c "
  install -m 644 /tmp/zenoh-public-key.gpg /etc/apt/keyrings/zenoh-public-key.gpg &&
  echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' > /etc/apt/sources.list.d/zenoh.list &&
  apt-get update &&
  apt-get install -y zenoh zenoh-backend-rocksdb python3-venv python3-pip
"
```

WSL2 / 一部の minimal Ubuntu では `python3-venv` が欠けているので同時に入れる。

### 3. mesh-mem を venv に install

```bash
cd ~/work/mesh-mem

# 既存 clone がある場合は WIP の有無を必ず確認してから fast-forward する。
# `inherit-wip` 系の前提と衝突するので、未コミット差分があるなら commit / stash / 別 worktree を選ぶ。
if [ -n "$(git status --porcelain)" ]; then
  echo "WARN: working tree dirty — resolve before pulling" && git status --short
  # 強制 pull はしない。dirty なら手動判断。
else
  git fetch origin main
  git checkout main
  git merge --ff-only origin/main
fi
# 特定 revision を狙うなら `git checkout <tag-or-sha>` に置き換える。

python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -e '.[dev,test]'
.venv/bin/mesh-mem --version   # 0.2.x を確認
```

### 4. zenohd config を生成

`config/zenohd_peer.json5.template` をベースに、新規 peer 用の config を作る。

**重要: 実 IP を含む runtime config は untracked path に置く**こと。repo 配下
（`config/`）に置くと、git に実 IP を commit してしまうリスクがあり、本リポジトリ
の placeholder 方針（commit には実 IP を入れない）と矛盾する。

```bash
mkdir -p ~/.config/mesh-mem
cat > ~/.config/mesh-mem/zenohd_<peer_name>.json5 <<CFG
{
  mode: "router",
  listen: {
    endpoints: [
      "tcp/127.0.0.1:7447",
      "tcp/<NEW_PEER_IP>:7447",
    ],
  },
  connect: {
    endpoints: [
      "tcp/<EXISTING_PEER_IP>:7447",
      // 直接到達できる既存 peer をすべて。1 個でも transit で全 mesh に届く
    ],
  },
  timestamping: {
    enabled: { router: true, peer: true, client: true },
  },
  plugins: {
    storage_manager: {
      volumes: { rocksdb: {} },
      storages: {
        agent_mem: {
          key_expr: "mem/**",
          strip_prefix: "mem",
          replication: { interval: 10.0, sub_intervals: 5, hot: 6, warm: 30, propagation_delay: 250 },
          volume: { id: "rocksdb", dir: "agent_mem", create_db: true },
        },
      },
    },
  },
}
CFG
mkdir -p ~/.local/share/mesh-mem
```

### 5. 既存 peer の listen を拡張（必要時のみ）

既存 router が新規 host から到達可能な IP に listen していない場合のみ。例：

- Home zenohd が ppp0 (192.168.134.28) にしか listen しておらず、新規 WSL2 (192.168.3.29) からは LAN IP (192.168.3.12) でしか到達できないケース。

そのときは：

```bash
# 既存 host で（dispatcher が SSH や直接実行）
# 1. config を runtime 用に untracked path にコピー
mkdir -p ~/.config/mesh-mem
cp config/zenohd_home.json5 ~/.config/mesh-mem/zenohd_home_local.json5
# 2. listen に新 IP を追記、placeholder があれば実 IP に置換
#    (このファイルは untracked、実 IP を含んでよい)

# 3. 既存 zenohd を厳密 PID 指定で停止（broad pkill は危険）
#    複数 zenohd が並走しているケースに備え、必ず PID を列挙してから cmdline で
#    意図したものだけを選別し、目視承認を経て kill する。
#
# 対象 config の "識別子" を先に決める。これは cmdline 検証と起動の両方で使う。
# 既存 (committed) config と _local 版のどちらでも match させたいので extended
# regex で両許容する。peer name によって調整する。
TARGET_CONFIG_REGEX='zenohd_home(_local)?\.json5'
TARGET_CONFIG_NEW=~/.config/mesh-mem/zenohd_home_local.json5   # 再起動先

ZENOH_PIDS=$(pgrep -x zenohd || true)
if [ -z "$ZENOH_PIDS" ]; then
  echo "no zenohd running"; return 1 2>/dev/null || exit 1
fi
echo "=== zenohd PIDs and cmdlines ==="
for pid in $ZENOH_PIDS; do
  printf "PID %s : " "$pid"
  tr '\0' ' ' < /proc/"$pid"/cmdline; echo
done
echo "=== select the PID whose cmdline matches: ${TARGET_CONFIG_REGEX} ==="
echo "    (after stop, restart will use config: ${TARGET_CONFIG_NEW})"
# 例: 上記出力から目視で選び、TARGET_PID にだけセットする
TARGET_PID=<上記出力から目視で選んだ PID>   # 例: 2392214

# safety: 1 PID であること、対象 config regex が cmdline に含まれることを最終確認
test -n "$TARGET_PID" && [ "$(echo "$TARGET_PID" | wc -w)" -eq 1 ] || { echo "TARGET_PID malformed"; exit 1; }
tr '\0' ' ' < /proc/"$TARGET_PID"/cmdline | grep -Eq "$TARGET_CONFIG_REGEX" \
  || { echo "selected PID cmdline does not match $TARGET_CONFIG_REGEX"; exit 1; }

kill -TERM "$TARGET_PID"
# 終了待ち（最長 10 秒）。SIGKILL fallback が必要なら追記
for _ in $(seq 1 20); do
  kill -0 "$TARGET_PID" 2>/dev/null || break
  sleep 0.5
done

# 4. runtime config で再起動（上で定義した TARGET_CONFIG_NEW を使う）
ZENOH_BACKEND_ROCKSDB_ROOT=$HOME/.local/share/mesh-mem \
  nohup zenohd -c "$TARGET_CONFIG_NEW" \
  > /tmp/zenohd_home.log 2>&1 &
disown
sleep 4
ss -tlnp | grep 7447   # 新 IP も listen に含まれていることを確認
```

注意：
- committed config はセキュリティのため placeholder のままにしておくこと。実 IP は untracked パスに置く。
- 複数 zenohd プロセスが並存する環境（smoke test 中、debug session 残留など）では `pgrep -x zenohd` が複数返る。`pkill -f` は誤爆するので、必ず PID + cmdline 検証経由で 1 個ずつ判断する。

### 6. 新規 peer 起動

```bash
export ZENOH_BACKEND_ROCKSDB_ROOT=$HOME/.local/share/mesh-mem
nohup zenohd -c ~/.config/mesh-mem/zenohd_<peer_name>.json5 > /tmp/zenohd_<peer_name>.log 2>&1 &
disown
sleep 5
ss -tlnp | grep 7447                 # listen 確認
nc -zv <EXISTING_PEER_IP> 7447       # 既存 peer への TCP 疎通確認
```

### 7. 双方向 replication 確認

`status` の per-pc breakdown だけでは「既存→新規」が確実に届いた証拠としては
弱い（observer は SQLite キャッシュに依存するし、initial alignment 進行中の
途中値かもしれない）。**双方向と言うなら、両向きの save → 反対側からの取得**
を 2 ペアで明示的に確認する。

#### 7-a. new → existing 方向

新規 peer 側で：

```bash
export MESH_MEM_AGENT_FAMILY=claude
export MESH_MEM_CLIENT_ID=<NEW_CLIENT_ID>

# 1. status の per-pc breakdown で既存 peer の pc_id が visible であることを確認
#    （initial alignment ~30s 待つ。途中値でもかまわないが peer が消えるなら NG）
.venv/bin/mesh-mem status

# 2. 新規 peer で save
NEW2EXIST_ID=$(.venv/bin/mesh-mem save "new->existing peer test from <peer_name>" --project mesh-add-test 2>&1 | grep -oE '[0-9a-f]{32}')
echo "new peer saved: $NEW2EXIST_ID"
sleep 8
```

既存 peer 側（dispatcher / SSH 経由）で：

```bash
mesh-mem get-memory $NEW2EXIST_ID
# 期待: 全 metadata 揃って取れる。agent: claude/<NEW_CLIENT_ID>
```

#### 7-b. existing → new 方向

既存 peer 側で：

```bash
EXIST2NEW_ID=$(mesh-mem save "existing->new peer test for <peer_name>" --project mesh-add-test 2>&1 | grep -oE '[0-9a-f]{32}')
echo "existing peer saved: $EXIST2NEW_ID"
sleep 8
```

新規 peer 側で：

```bash
.venv/bin/mesh-mem get-memory $EXIST2NEW_ID
# 期待: 全 metadata 揃って取れる。agent は既存 peer の client_id
```

両方向で `get-memory` 成功すれば双方向 replication 確立。
片方だけ通る場合は片側ファイアウォール / NAT / config の listen 不備を疑う。

### 8. （任意）Claude Code MCP 登録

新規 peer 上に Claude Code が入っていればそのホストで MCP 使えるようにする：

```bash
which claude && claude --version

claude mcp add mesh_mem -s user \
  -e ZENOH_CONNECT=tcp/127.0.0.1:7447 \
  -e MESH_MEM_AGENT_FAMILY=claude \
  -e MESH_MEM_CLIENT_ID=<NEW_CLIENT_ID> \
  -- ~/work/mesh-mem/.venv/bin/mesh-mem-mcp

claude mcp list   # ✓ Connected を確認（初回は ~10s 待ってリトライ必要なことあり）
```

## トラブルシュート

| 症状 | 原因候補 | 対処 |
|------|---------|------|
| `ping <existing_peer_IP>` が通らない | LAN segment が違う、ルート無し | 別の既存 peer IP（Tailscale 経由など）を使うか、ルーティング修正 |
| `nc -zv ... 7447` 通らない | 既存 peer がその IP で listen していない | ステップ 5 で listen 拡張 |
| `mesh-mem status` が hang する | ストアが巨大で Zenoh full-scan timeout | `MESH_MEM_DISABLE_INDEX` を unset、SQLite index sidecar を使わせる。それでも遅ければ `MESH_MEM_STATE_DIR=/tmp/<fresh>` で fresh state |
| `mesh_ready: waiting` が消えない | initial alignment 進行中（cold era だと数分） | 30 秒〜数分待つ。`status` の件数が増えていけば動いている |
| MCP `Failed to connect` | fastmcp 初回起動 race | 10 秒待って `claude mcp list` 再実行 |

## 出力（呼び出し元へ報告）

- 新規 peer の IP / hostname / pc_id
- 既存 peer 側の listen 拡張があったか / なかったか
- replication 双方向 verify 結果（save した ID と既存 peer 側 get-memory の成功）
- MCP 登録の結果（`✓ Connected`）
- 残課題（auto-start 未設定、untracked config の永続化、など）

## 派生課題（しばしば次ステップで必要になる）

- **auto-start**：systemd-user unit / cron @reboot / WSL2 startup script。デフォルトでは入れない。
- **永続 config**：runtime config を `/tmp` ではなく `~/.config/mesh-mem/zenohd_<role>_local.json5` などに移し、起動スクリプトで参照させる。
- **Office / 他 peer の v0.2.x 揃え**：CLI バージョンが peer 間で不一致だと `get-memory` 等が一方からしか叩けない。
