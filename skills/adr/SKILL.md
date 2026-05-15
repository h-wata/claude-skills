---
name: adr
description: Architecture Decision Records (ADR) の作成・管理スキル。`/adr` で新規ADR作成、`/adr list` で一覧表示、`/adr supersede 0001` で既存ADRの置換。設計判断の「なぜ」を不変のドキュメントとして記録する。アーキテクチャの意思決定、設計方針の記録、技術選定の理由の文書化に使う。
---

# ADR Skill

プロジェクトの設計判断を Architecture Decision Records (ADR) として記録・管理する。

ADRの価値は「不変性」にある。一度書いたら書き換えず、変更時は新しいADRで置換(supersede)する。これによりgrepで発見しても安全で、ステータスで現在有効な決定を構造的に判別できる。

## 使い方

```
/adr                    → 対話的に新規ADR作成
/adr list               → 既存ADRの一覧表示（番号・タイトル・ステータス）
/adr supersede 0001     → ADR-0001を置換する新ADRを作成
```

## 配置先（ADR ルートの解決）

新規 ADR は **Obsidian vault 優先** で配置する。リポジトリには push しないことを既定とする（個人作業ファイルとして扱う）。

解決順序：

1. **vault モード（既定）**: `~/Documents/my_obsidian/projects/<basename(cwd)>/ADR/`
   - vault プロジェクトディレクトリ (`~/Documents/my_obsidian/projects/<basename(cwd)>/`) が存在し書き込み可能なら、その下の `ADR/` を ADR ルートとする
   - `ADR/` サブディレクトリが無ければ作成する
2. **アクセス不可なら `/add-dir` を依頼して中断**:
   - vault プロジェクトディレクトリへの書き込みで permission エラーになる場合、`additionalDirectories` に未登録の可能性が高い
   - ユーザに対して以下のように案内し、処理を中断する：
     ```
     vault への書き込みができません。次を実行してから /adr を再実行してください：
       /add-dir ~/Documents/my_obsidian/projects/<basename(cwd)>/
     ```
   - vault プロジェクトディレクトリ自体が未作成の場合は、Obsidian で先にプロジェクトを作るか、`mkdir -p` で作るか案内する
3. **`docs/adr/` フォールバック（明示要求時のみ）**:
   - ユーザが「team-shared にしたい」「リポジトリに置きたい」と明示した場合のみ、リポジトリの `docs/adr/` に書く
   - 既定では `docs/adr/` が存在しても **書き込み先には選ばない**。あくまで vault 優先

`docs/adr/` がリポジトリに既存で、かつ vault にも ADR がある「移行中」状態では、`/adr list` で両方を表示する（後述）。

## ADRテンプレート

```markdown
# ADR XXXX: タイトル

- **Status**: Accepted
- **Date**: YYYY-MM-DD
- **Supersedes**: なし
- **Related**: [[XXXX-related-adr]]（任意、Obsidian wikilink 推奨）

## Context

なぜこの決定が必要だったか。背景・制約・選択肢を記述する。

## Decision

何を決定したか。採用した方針を明確に記述する。

## Consequences

- **良い点**: この決定によるメリット
- **悪い点**: この決定によるデメリットやトレードオフ
```

- 見出しは `# ADR XXXX:` (半角スペース、ハイフン無し)。vault 既存 ADR の表記に合わせる
- メタ行は `**Status**` 等の bold 形式
- `Related` は Obsidian wikilink `[[file-basename]]` で書くと vault 内で相互参照しやすい

## 実行フロー

### `/adr`（新規作成）

1. ADR ルートを解決する（上記「配置先」セクション参照）。vault アクセス不可なら `/add-dir` 案内で中断
2. ADR ルートを走査し、既存 ADR の最大番号を取得して次の番号を決定する
3. AskUserQuestion で以下をヒアリングする（まとめて聞く、1個ずつ聞かない）:
   - どんな設計判断を記録したいか
   - なぜその判断が必要だったか（背景・制約）
   - 他にどんな選択肢があったか
   - 採用した方針とその理由
4. git log や関連コードを調査し、判断の背景を補完する
5. ADRファイルを生成し、ユーザーに内容を確認してもらう
6. 確認後、ファイルを `<ADRルート>/XXXX-descriptive-name.md` に書き出す

ファイル名の `descriptive-name` 部分は、決定内容を端的に表す英語のケバブケースにする（例: `queryable-command-pattern`, `grpc-deadline-with-cache-fallback`）。

### `/adr list`

1. vault の ADR ルートを走査する
2. **加えて** リポジトリ直下の `docs/adr/` が存在すれば、そこも走査する（移行中の表示用）
3. 各ファイルから番号・タイトル・ステータスを抽出する
4. テーブル形式で一覧表示する。ソースが混在する場合は Source 列を付ける

```
| #    | タイトル                         | Status     | Source     |
|------|----------------------------------|------------|------------|
| 0001 | Bridge は temi-app に住む          | Accepted   | vault      |
| 0002 | REST から integration への pivot   | Accepted   | vault      |
| 0003 | RobotGate 状態機械                  | Accepted   | vault      |
| 0099 | (legacy) docs/adr/ にだけ残るもの   | Accepted   | docs/adr/  |
```

Source 列はソースが 1 箇所だけの場合は省略してよい。

### `/adr supersede XXXX`

1. ADR ルート（vault と `docs/adr/` 両方）から ADR XXXX を探して読み込み、内容を把握する
2. AskUserQuestion で「何が変わったのか、なぜ変えるのか」をヒアリングする
3. 新しい ADR を作成する（`**Supersedes**: ADR XXXX` を記載）
   - 新規 ADR は vault モードのルールに従って vault に書く（旧 ADR が `docs/adr/` にあっても新規は vault）
4. 旧 ADR の Status 行のみを `Superseded by ADR YYYY` に更新する
   - これがADRの不変原則における唯一の例外

## 重要なルール

- **不変原則**: 既存ADRの内容は絶対に書き換えない。唯一の例外はSuperseded時のStatus更新のみ
- **vault 優先**: 既定で vault に書く。リポジトリ (`docs/adr/`) には push しない方針を尊重する
- **`/add-dir` 依頼**: vault に書けないときは無理に `docs/adr/` にフォールバックせず、ユーザに `/add-dir` を依頼して中断する
- **ヒアリングは必ずAskUserQuestionを使う**: 質問は2〜4個にまとめて聞く
- **背景の調査**: git log やコードを読んで、ユーザーの説明を補完する。ユーザーが語らなかった技術的背景も拾う
- **日本語で対話**: ユーザーとのやりとりは日本語。ADR本文も日本語でOK（タイトルのケバブケース部分のみ英語）
- **過剰に書かない**: Context/Decision/Consequencesの各セクションは簡潔に。長文より「なぜ」が伝わることが重要
