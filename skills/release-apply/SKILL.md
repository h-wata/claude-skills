---
name: release-apply
description: drafts に揃ったリリースノート / CHANGELOG / migration を実プロジェクトに反映してタグ・GitHub Release・関連 Issue close まで実行する。CHANGELOG 置換 → version bump → tests → commit + tag + push --tags → gh release create → Issue close を 1 タスクで完結。
allowed-tools: Bash, Read, Edit, Write
---

事前に user 承認を得たリリース drafts を、実プロジェクトに apply するスキル。

## 使うタイミング

- リリース drafts（CHANGELOG セクション / release notes / migration note）が
  別の場所に揃っており、内容が user に確認・承認済み
- これから tag を打って GitHub Release を作成する直前

drafts がまだの場合や user 承認前は **このスキルを呼ばない**。先に drafts を仕上げて
user に提示し、明示の OK をもらってから実行する。

## 引数

`$ARGUMENTS` で以下をパイプ区切り or YAML 風で指定:

- `repo`: GitHub リポジトリ（例: `h-wata/mesh-mem`）
- `version`: リリースバージョン（例: `0.2.0`）
- `date`: リリース日（例: `2026-05-01`）
- `drafts_dir`: drafts ディレクトリの絶対パス（例: `/home/gisen/work/tmux-multi-agents/queue/drafts/v0.2.0/`）
  - 配下に `CHANGELOG_<ver>_section.md` / `release_notes_<ver>.md` / `migration_<prev>_to_<ver>.md` を期待
- `workspace`: プロジェクトの作業ディレクトリ（例: `/home/gisen/work/mesh-mem`）
- `prep_issue`: release prep を tracking している Issue 番号（例: `6`、close 対象）

引数が不足している場合は user に確認する。

## 手順

### 1. 事前確認

```bash
cd <workspace>
git fetch origin main
git status   # working tree が clean であること
git log --oneline -1   # HEAD を控える
grep '^version' pyproject.toml   # 現バージョンを控える
```

working tree に modified / untracked ファイルがあれば停止して user 報告。
**release commit に余計な変更を巻き込まない**ため、ここは厳格に。

### 2. CHANGELOG.md の更新

drafts の `CHANGELOG_<ver>_section.md` で `[Unreleased]` セクションを置換し、
新たな空の `[Unreleased]` を上に追加する。

```python
# python3 で安全に置換
from pathlib import Path
import re

changelog = Path('CHANGELOG.md')
draft = Path('<drafts_dir>/CHANGELOG_<ver>_section.md')

current = changelog.read_text()
draft_text = draft.read_text()

marker = '## [Unreleased]'
if marker not in current:
    raise SystemExit('marker not found, abort')

header = current.split(marker)[0]
rest = current[current.index(marker):]

# [Unreleased] 直下から次の "## [" までを削除（既存の WIP entries）
m = re.search(r'^## \[(?!Unreleased)', rest, flags=re.MULTILINE)
legacy = rest[m.start():] if m else ''

new = (
    header
    + '## [Unreleased]\n\n'
    + draft_text.rstrip() + '\n\n'
    + legacy
)
changelog.write_text(new)
```

置換結果を `head -30 CHANGELOG.md` で目視確認。

### 3. version bump

```bash
# pyproject.toml
sed -i "s/^version = \"<old>\"/version = \"<new>\"/" pyproject.toml
grep '^version' pyproject.toml   # 確認

# 他に version を持つファイル（__init__.py の __version__ 等）があれば同じ要領で
```

### 4. テスト実行

```bash
PYTHONPATH=src python -m pytest tests/ -x -q 2>&1 | tail -10
```

**テスト失敗時は release を中止** し、user に報告。lint / format も走らせて
事前に通しておく:

```bash
ruff check --fix src/ tests/ 2>/dev/null || true
yapf -i src/<package>/*.py 2>/dev/null || true
```

### 5. commit + tag + push

```bash
git add CHANGELOG.md pyproject.toml
git commit -m "$(cat <<'EOF'
release: v<version>

<3-5 行の Highlights を箇条書き>

See CHANGELOG.md and release notes for full details.
EOF
)"

git tag v<version>
git push origin main --tags
```

**Co-Authored-By 行は付けない**（明示的に user 指示があった場合のみ）。
**tag 重複時 (v<version> が既存) は中止** して user 報告。

### 6. GitHub Release 作成

```bash
gh release create v<version> \
  --repo <repo> \
  --title "<project> v<version>" \
  --notes-file <drafts_dir>/release_notes_<version>.md
```

戻り値の URL を控える。

### 7. release prep Issue を close

```bash
gh issue close <prep_issue> \
  --repo <repo> \
  --comment "v<version> released. See https://github.com/<repo>/releases/tag/v<version>"
```

### 8. verify

```bash
git tag --list | grep "v<version>"
gh release view v<version> --repo <repo> --json url,tagName,name -q '.url, .tagName, .name'
gh issue view <prep_issue> --repo <repo> --json state -q '.state'
grep '^version' pyproject.toml
head -15 CHANGELOG.md
```

すべての項目で期待値を確認。

### 9. 報告

以下を含めて報告:

- commit hash
- tag name
- GitHub release URL
- prep Issue close 結果
- pyproject.toml の version
- pytest 結果
- 想定外があれば（CHANGELOG 構造の崩れ、merge conflict、tag 重複など）

## 注意

- **これは外部投稿アクション**: tag push / GitHub Release / Issue close すべて公開操作。
  drafts と引数は user 承認済みのものだけを使う
- **テスト失敗 / lint 不通過時は中止**: 「とりあえず release」は厳禁
- **rebase / push の前に必ず `git fetch`**: 並行作業がある環境では特に重要
- **CHANGELOG.md の構造が崩れた場合**: commit する前に手動修正、または `git restore`
- **Co-Authored-By は付けない**: 明示指示がある場合のみ
- **draft 自体は残す**: apply 後も drafts ファイルは履歴として保管。新規作成・移動はしない
