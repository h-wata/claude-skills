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

## 配置先

作業ディレクトリの `docs/adr/` に配置する。ディレクトリが存在しなければ作成する。

## ADRテンプレート

```markdown
# ADR-XXXX: タイトル

- Status: Accepted
- Date: YYYY-MM-DD
- Supersedes: なし

## Context

なぜこの決定が必要だったか。背景・制約・選択肢を記述する。

## Decision

何を決定したか。採用した方針を明確に記述する。

## Consequences

- **良い点**: この決定によるメリット
- **悪い点**: この決定によるデメリットやトレードオフ
```

## 実行フロー

### `/adr`（新規作成）

1. `docs/adr/` を走査し、既存ADRの最大番号を取得して次の番号を決定する
2. AskUserQuestion で以下をヒアリングする（まとめて聞く、1個ずつ聞かない）:
   - どんな設計判断を記録したいか
   - なぜその判断が必要だったか（背景・制約）
   - 他にどんな選択肢があったか
   - 採用した方針とその理由
3. git log や関連コードを調査し、判断の背景を補完する
4. ADRファイルを生成し、ユーザーに内容を確認してもらう
5. 確認後、ファイルを `docs/adr/XXXX-descriptive-name.md` に書き出す

ファイル名の `descriptive-name` 部分は、決定内容を端的に表す英語のケバブケースにする（例: `queryable-command-pattern`, `grpc-deadline-with-cache-fallback`）。

### `/adr list`

1. `docs/adr/` 内の全ADRファイルを走査する
2. 各ファイルから番号・タイトル・ステータスを抽出する
3. テーブル形式で一覧表示する

```
| #    | タイトル                         | Status     |
|------|----------------------------------|------------|
| 0001 | Queryable-Based Command Pattern  | Accepted   |
| 0002 | コマンドコンテキストとテレメトリ分離 | Accepted   |
| 0003 | gRPCデッドライン+キャッシュ        | Superseded |
```

### `/adr supersede XXXX`

1. 指定されたADRを読み込み、内容を把握する
2. AskUserQuestion で「何が変わったのか、なぜ変えるのか」をヒアリングする
3. 新しいADRを作成する（`Supersedes: ADR-XXXX` を記載）
4. 旧ADRの Status 行のみを `Superseded by ADR-YYYY` に更新する
   - これがADRの不変原則における唯一の例外

## 重要なルール

- **不変原則**: 既存ADRの内容は絶対に書き換えない。唯一の例外はSuperseded時のStatus更新のみ
- **ヒアリングは必ずAskUserQuestionを使う**: 質問は2〜4個にまとめて聞く
- **背景の調査**: git log やコードを読んで、ユーザーの説明を補完する。ユーザーが語らなかった技術的背景も拾う
- **日本語で対話**: ユーザーとのやりとりは日本語。ADR本文も日本語でOK（タイトルのケバブケース部分のみ英語）
- **過剰に書かない**: Context/Decision/Consequencesの各セクションは簡潔に。長文より「なぜ」が伝わることが重要
