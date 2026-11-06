---
name: drawio
description: md の構成・画面一覧・処理の流れなどから draw.io（.drawio）の図を作り、draw.io Desktop の CLI で PNG / SVG / PDF に書き出して自分で確かめる。「draw.io で図にして」「構成図を描いて」「画面遷移図を作って」「.drawio を PNG にして」といった場面で使う。MCP は使わず XML を直接書く。
---

# drawio — XML を直接書いて図にし、書き出して確かめる

`.drawio` の中身は XML。エージェントが直接書き、draw.io Desktop の CLI で書き出す。
書き出した画像を自分で読んで確かめるまでが手順。

## 1. 図の元を決める

- 元になる md（構成・画面一覧・手順など）があれば、それを正本にする。図に書く箱と線は
  md にあるものだけにし、推測で足さない
- 元がなければ、何を箱にして何を線にするかを先に箇条書きでユーザーに見せてから描く
- 保存先は特に指定がなければ元の md と同じフォルダに `<名前>.drawio`

## 2. XML を書く

ひな形（箱2つと線1本）:

```xml
<mxfile>
  <diagram id="main" name="図の名前">
    <mxGraphModel dx="800" dy="600" grid="1" gridSize="10" page="1" pageWidth="1169" pageHeight="827">
      <root>
        <mxCell id="0"/>
        <mxCell id="1" parent="0"/>
        <mxCell id="a" value="SC-001&#xa;ログイン" style="rounded=1;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf;" vertex="1" parent="1">
          <mxGeometry x="40" y="120" width="140" height="60" as="geometry"/>
        </mxCell>
        <mxCell id="b" value="SC-002&#xa;一覧" style="rounded=1;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf;" vertex="1" parent="1">
          <mxGeometry x="300" y="120" width="140" height="60" as="geometry"/>
        </mxCell>
        <mxCell id="e1" value="認証成功" style="endArrow=classic;html=1;" edge="1" parent="1" source="a" target="b">
          <mxGeometry relative="1" as="geometry"/>
        </mxCell>
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>
```

書き方の決まり:

- `id="0"` と `id="1"` の2つは必ず置き、他の要素は `parent="1"` にする
- id は意味のある短い英字（`login`, `db`, `e1`）にし、重複させない
- 線（`edge="1"`）の `source` / `target` は、実在する箱の id にする。**存在しない id を
  指した線はエラーにならず、書き出すと黙って消える**
- 改行は `&#xa;`、`&` `<` `>` `"` は `&amp;` `&lt;` `&gt;` `&quot;` に置き換える
- 座標は 10 の倍数にそろえ、箱どうしは横 120 以上・縦 80 以上あける。重なりを避けるため、
  左から右（または上から下）へ流れる順に並べる
- 戻りの線は `<Array as="points">` で迂回点を入れ、他の線や箱と重ねない
- 色は種類ごとに1色に決める（例: 画面 #dae8fc/#6c8ebf、外部 #f8cecc/#b85450、
  データ #d5e8d4/#82b366）。凡例が要るなら右下に置く
- 複数ページは `<diagram>` を並べる

## 3. 機械チェック

```bash
python3 ~/.claude/skills/drawio/scripts/check.py <図>.drawio
```

XML として読めるか、id の重複、線の接続先が存在するかを調べる。NG があれば直してから次へ。

## 4. 書き出す

```bash
drawio -x -f png -e --ozone-platform=headless --disable-gpu -o <図>.drawio.png <図>.drawio
```

- `-f` は `png` / `svg` / `pdf`。`-e` 付きの PNG は図のデータを埋め込むので、draw.io で
  そのまま開いて編集できる
- `--ozone-platform=headless --disable-gpu` は画面のない環境（SSH 先など）でも動かすため
- XML が壊れていると終了コード 1 と `Export failed` になる
- `drawio` が見つからなければ、draw.io Desktop（Linux は AppImage、Windows / Mac は
  ポータブル版でもよい）を入れてもらうようユーザーに伝える。書き出しを飛ばして
  `.drawio` だけ渡してもよい

## 5. 書き出した画像を読んで確かめる

PNG を Read で開き、次を確かめる。直したら 3 からやり直す。

- md にある箱と線がすべて出ているか（消えた線がないか）
- 文字が箱からはみ出ていないか、箱や線が重なっていないか
- 線のラベルがどの線のものか読み取れるか

最後に、作ったファイルのパスと、md と図の対応で気になった点を短く報告する。
