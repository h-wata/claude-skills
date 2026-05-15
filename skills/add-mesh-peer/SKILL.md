---
name: add-mesh-peer
description: 既存 mesh-mem (Zenoh) メッシュに新しいホストを peer として参加させる。zenoh apt インストール → venv → config 生成 → 既存 peer の listen 拡張 → 起動 → 双方向 replication 確認 → 任意で Claude Code MCP 登録、までを一気通貫。
allowed-tools: Bash, Read, Edit, Write, Glob, Grep
---

# add-mesh-peer

mesh-mem の N-host メッシュに 1 ホスト追加するときの定番手順。WSL2 への 3rd peer 追加（2026-05-04）で確立、hub-spoke topology 化（2026-05-10 検証）でアップデート。

## トポロジ前提：1 hub + N spokes

新 peer 追加で「既存 peer 全部に手を入れる」のは不要。基本パターンは:

- **hub** (常時稼働の 1 台、典型的には home) が、どの spoke からでも届く IP を `listen` に集約する。
- **spoke** (新規 peer 含むその他) は hub に向けて connect 1 本だけ書く。spoke 同士の直接 link は不要 (Zenoh router transit が双方向通信を成立させる)。
- 新規 spoke 追加時、**既存 peer の config は touch しない / restart しない**。これは 2026-05-10 の 3-PC 実機実験で実証済み (`docs/poc-reports/topology-2026-05-10.md`)。

例外: hub の listen に新 spoke から到達可能な IP が無い場合 **だけ**、hub 側を 1 度 restart して listen 拡張する (これは hub の listen 集約を一度だけ更新する話で、毎回の作業ではない)。

## 前提条件

- hub が 1 台動作中で、新規 host から hub の listen IP のいずれか 1 つに TCP/7447 が届く
- 新規 host で `sudo`、`python3 >= 3.10`、`apt-get` が使える
- 新規 host から hub へ：ICMP / TCP/7447 の outbound が通ること（先に `ping` / `nc -zv` で確認）

## いつ使うか

- 個人利用の N-device mesh を 1 台ずつ拡張するとき
- 既存 mesh と隔離したテスト peer を追加するとき
- WSL2 / 別ホスト / VPS を mesh に組み込むとき

## 入力（必ず確認すること）

| 項目 | 例 | 用途 |
|------|------|------|
| `NEW_PEER_IP` | 192.168.3.29 | 新規 host の LAN IP（spoke が listen する側、MCP client が同 host から dial する用） |
| `NEW_PEER_HOSTNAME` | wsl-desktop | log や naming で識別 |
| `HUB_IP` | 192.168.3.12 | 新規 host が outbound で connect する hub (常時稼働 peer)。Zenoh transit で全 spoke のデータに到達する |
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

`config/zenohd_peer.json5.template` をベースに、新規 peer (spoke) 用の config を作る。

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
  // spoke は hub に dial 1本だけ。Zenoh transit で他 spoke のデータも届く。
  // 既存 peer の config は touch しない。
  connect: {
    endpoints: [
      "tcp/<HUB_IP>:7447",
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

WSL2 host の場合は loopback / port 衝突に注意。§9 を参照。

### 5. hub の listen 拡張（**必要時のみ**、1 度きりの作業）

通常は新 peer 追加で hub に手を入れない。例外として、**hub の listen に新 spoke
から到達可能な IP が無い** 場合のみ、この 1 度だけ hub を restart する。例：

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

### 9. WSL2 ホストを peer にする場合（mirrored networking 特有の手順）

WSL2 (Windows host 上の Ubuntu) を spoke にするときの追加手順。2026-05-10 の実機検証で詰まった点をまとめる。

#### 9-a. listen は `LAN IP:7448`、loopback は外す

WSL2 の `localhost` / `127.0.0.1` および LAN IP (eth1 の `192.168.x.x`) は **mirrored networking で Windows host とネットワークスタックを共有** する。Windows host 側に何か listen していると、WSL2 側 zenohd の bind が `Address already in use (os error 98)` で落ちる。**これは `ss -tlnp` / `lsof -iTCP:7447` / `/proc/net/tcp` のいずれにも表示されない** (WSL2 内では Windows host の socket が見えない) ので、原因特定が難しい。

回避策 — runtime config で:
- `tcp/127.0.0.1:7447` の行を **削除** する (loopback は localhost MCP 用だが、WSL2 は Windows 側から見れば eth1 経由で同じ host なので不要)
- LAN IP の port を `7447` ではなく **`7448` にずらす** (Windows host 側の何が握っているかは詰めないで OK、port を逃せば早い)

```jsonc
listen: {
  endpoints: [
    "tcp/<NEW_PEER_IP>:7448",   // 7447 ではなく 7448
  ],
},
connect: {
  endpoints: [
    "tcp/<HUB_IP>:7447",         // hub 側はそのまま 7447
  ],
},
```

mesh-mem CLI を WSL2 上で叩くときは `ZENOH_CONNECT=tcp/<NEW_PEER_IP>:7448` を export する (default の `tcp/127.0.0.1:7447` だと当然繋がらない)。

#### 9-b. clock skew を 500ms 未満に追い込む（manual step）

Zenoh の replication は **timestamp tolerance 500ms** がハードコード (`exceeding delta 500ms is rejected` で reject ログが出る)。WSL2 は **Windows host の hibernation 後に clock が数秒〜数十秒ずれる** のが定番症状。`systemd-timesyncd` は active でも slew (徐々に補正) しかせず即時 step しない。

```bash
# 0. chrony / ntpdate / sntp が無いか確認 (Ubuntu minimal だと全部欠けてる)
which ntpdate chronyd sntp || dpkg -l | grep -E 'chrony|ntpdate|sntp' || echo "no step tools"

# 1. timesyncd を一時停止
sudo timedatectl set-ntp false

# 2. hub から WSL2 へ ssh してると round-trip latency が ~1s 出るので、
#    bias 込みで set する。dispatcher (hub) 側で:
HUB_NOW=$(date -u +%s.%3N)
TARGET=$(awk -v t="$HUB_NOW" 'BEGIN{printf "%.3f", t+1.0}')   # +1.0s bias
# ↓ tmux send-keys や ssh 越しに WSL2 で実行
sudo date -u -s "@$TARGET"

# 3. 並走で hub と WSL2 の date を読み比べ、skew が ±500ms 以内に入るまで bias 調整して再 set
ssh wsl2 'date -u +%s.%3N'   # vs HUB の date -u +%s.%3N
# → 差分が WSL2 - HUB であれば、その差分を引いて再 step

# 4. step が決まったら timesyncd を戻す (徐々に slew で 微補正してくれる)
sudo timedatectl set-ntp true
```

**実例 (2026-05-10)**: 初回 `+1.0s bias` で skew -1.0s → `+1.0s` 再 step で skew -14ms に収束。許容 500ms に余裕で収まる。

#### 9-c. zenohd log で「同期できた」を確認

```bash
# clock 修正前後で error 数を比較
tail -200 /tmp/zenohd_<peer>.log | grep -c "exceeding delta"   # 修正前: 数百
# 修正後 30 秒以内
awk -v cutoff="$(date -u -d '30 seconds ago' +%Y-%m-%dT%H:%M:%S)" \
  '$0 ~ /exceeding delta/ && $1 > cutoff {c++} END {print c+0}' /tmp/zenohd_<peer>.log
# → 0 が出れば clock skew 解消
```

ステップ 7 (双方向 replication 確認) はこの後に実施する。

## トラブルシュート

| 症状 | 原因候補 | 対処 |
|------|---------|------|
| `ping <hub_IP>` が通らない | LAN segment が違う、ルート無し | 別の hub listen IP（Tailscale 経由など）を使うか、ルーティング修正 |
| `nc -zv <hub_IP> 7447` 通らない | hub がその IP で listen していない | ステップ 5 で hub の listen を 1 度だけ拡張 |
| WSL2 で `Address already in use (os error 98)` | mirrored networking で Windows host stack を共有 | ステップ 9-a で port 7448 + loopback 削除 |
| zenohd log に `exceeding delta 500ms is rejected` 連発 | host 間 clock skew > 500ms (replication 不可) | ステップ 9-b で manual step (WSL2 hibernation 後の典型症状) |
| `mesh-mem get-memory` が hub から timeout | hub の store が大きく `mem/obs/**` wildcard scan が遅い | mesh-mem 側の既知課題 (`store.py:543`、別 issue 参照)。topology 起因ではない。spoke 側からの get-memory は通る |
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
