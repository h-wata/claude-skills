---
name: setup-claude-local
description: 現在のリポジトリに「ローカル管理 ADR + .claude/CLAUDE.md + Obsidian vault 連携」一式をセットアップする。リポジトリにレビュー対象外の個人作業ファイル (ADR / open_questions / Claude ローカル指示) を持ち込みたくないが、複数 PC 間で共有はしたい、というユースケースで使う。`/setup-claude-local` で起動。
---

# Setup Claude Local

リポジトリに個人作業用のローカル文脈 (ADR / open_questions / Claude プロジェクト指示) を直接置かず、**Obsidian vault に集約してリポジトリは綺麗に保つ** ためのセットアップを一括で行う。

## いつ使うか

- 新しいリポジトリで作業を始めるとき、個人の設計ノート (ADR) や Claude 用ローカル指示を置きたいが、リポジトリには push したくない
- 既存リポジトリの `docs/*/decisions/` `docs/adr/` のような場所にローカル ADR を置いていて、vault に移行したい
- 複数 PC でローカル文脈を共有したい (vault を git 同期している前提)

## 前提

- Obsidian vault が `~/Documents/my_obsidian/` 配下に存在し、git で同期されている
- vault の `projects/` 配下にプロジェクトごとのディレクトリを切る運用
- リポジトリ root で実行される

## 実行フロー

### Step 1: 状況把握

並列で以下を確認する:

1. `git rev-parse --show-toplevel` でリポジトリ root を取得 (skill 起動位置がリポジトリ内である前提)
2. リポジトリ名は root のディレクトリ名から取得 (例: `re-cafe-temi-app`)
3. 既存の `.claude/` `.gitignore` の状態を確認
4. 既存の ADR っぽいディレクトリ (`docs/adr/`, `docs/*/decisions/`, `docs/*/open_questions.md`) を `find` で探す
5. vault のプロジェクトディレクトリ `~/Documents/my_obsidian/projects/<repo-name>/` が既に存在するか確認

### Step 2: ユーザーへの確認

AskUserQuestion で以下を**まとめて**聞く:

- vault 内のプロジェクトディレクトリ名は `<repo-name>` で良いか? (デフォルト=repo名、`Other` で変更可)
- 既存の ADR / open_questions が見つかった場合: vault に**移動**するか、リポジトリに**残す**か、**スキップ**するか
- `.claude/CLAUDE.md` を新規作成するか (既存ならマージか上書きか)

質問が4個を超えるときは2回に分ける。

### Step 3: セットアップ実行

並列で以下を実行する:

#### 3.1 `.gitignore` への `.claude/` 追加

`.gitignore` を Read し、`.claude/` の行が無ければ末尾に追加:

```
# Claude Code のプロジェクトローカル設定・指示（個人作業用、レビュー対象外）
.claude/
```

#### 3.2 `.claude/CLAUDE.md` 生成

下記テンプレを書き出す (placeholder `<REPO_NAME>` を置換):

```markdown
# <REPO_NAME> プロジェクトローカル指示

このファイル自体および `.claude/` 配下は `.gitignore` 済み。個人作業用のローカル設定・指示として扱う。

## ADR と open_questions のローカル管理

ADR と `open_questions.md` は **個人 Obsidian vault で管理する設計コンテキスト**であり、リポジトリには含めない。

### 配置

vault: `~/Documents/my_obsidian/projects/<REPO_NAME>/`
- `ADR/` 配下に `NNNN-descriptive-name.md` 形式で配置
- `open_questions.md` で未解決事項を保持

vault は git で同期しているため、複数 PC 間で共有される。`.claude/settings.local.json` の `additionalDirectories` で `~/Documents/my_obsidian/projects` を登録済 → Claude からは直接読める。

### 守ること

- PR 本文・コミットメッセージ・**リポジトリに push されるドキュメント**で ADR 名を参照しない (レビュワーから辿れないリンク切れになる)
- ADR の情報を外部に出したいときは、リポジトリ内 docs に同等の記述を転記してから、そちらの節へリンクする
- `open_questions.md` も同様。レビュワーと共有したい未解決事項は PR 本文に直接書く

### ADR 運用 (vault 内)

- 新しい設計判断を ADR にまとめてよい (採番は連番)
- ADR は不変原則 (編集せず、新 ADR で Superseded にする)
- Obsidian の wikilink (`[[NNNN-name]]`) で相互参照できる
```

#### 3.3 `.claude/settings.local.json` 生成/更新

存在しなければ新規作成。存在すれば既存 JSON を Read して `additionalDirectories` に `/home/<user>/Documents/my_obsidian/projects` を追加 (重複チェック)。

最小新規テンプレ:

```json
{
  "permissions": {
    "additionalDirectories": [
      "/home/<user>/Documents/my_obsidian/projects"
    ]
  }
}
```

`<user>` は `$HOME` から導出。

#### 3.4 vault 側プロジェクトディレクトリ作成

```
~/Documents/my_obsidian/projects/<repo-name>/
├── ADR/
└── open_questions.md  (空 or 簡単なテンプレ)
```

既に存在する場合は触らない。

#### 3.5 既存 ADR / open_questions の移動 (ユーザーが「移動」を選んだ場合のみ)

- `docs/*/decisions/*` → `~/Documents/my_obsidian/projects/<name>/ADR/`
- `docs/*/open_questions.md` → `~/Documents/my_obsidian/projects/<name>/open_questions.md`
- 移動後、リポジトリ側の空ディレクトリは `rmdir` で削除

### Step 4: サマリ報告

実行したアクションを短く列挙する:

- `.gitignore`: `.claude/` を追加 (or "既に登録済み")
- `.claude/CLAUDE.md`: 作成 (or "既存を尊重してスキップ")
- `.claude/settings.local.json`: `additionalDirectories` 更新
- vault: `projects/<name>/ADR/` `open_questions.md` 作成
- 移動: `<source>` → `<dest>` (移動があった場合のみ)

`.gitignore` の差分はコミットするかどうかユーザーに判断を委ねる (skill 側ではコミットしない)。

## 注意事項

- vault が `~/Documents/my_obsidian/` ではないユーザーには対応していない。別パスを使う場合は SKILL.md を編集するか、対話的に上書きできるよう拡張すること
- ADR テンプレ自体は別 skill `/adr` で作成・管理する。本 skill は配置先のセットアップだけ責任を持つ
- `.claude/CLAUDE.md` は Claude Code が project root で自動読込するため、シンボリックリンク不要 (vault に置かない、各リポジトリで重複してよい)
- 既存 `.claude/CLAUDE.md` がある場合は上書きせず、追記候補を提示して終わる (ユーザーの編集に委ねる)
