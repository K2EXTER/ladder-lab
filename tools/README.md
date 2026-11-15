# PC 側の道具

## dmmlog.py — DMM の読み値を CSV に記録してグラフにする

試作 A のファーム(`../dmm/firmware/dmm_stageA`)とシリアルで話す。

準備(1回だけ):

```
pip install pyserial matplotlib
```

**Arduino IDE のシリアルモニタは閉じてから使う**(同じポートは1つのプログラムしか開けない)。

| やりたいこと | コマンド |
|---|---|
| 1回ごとの読み値を 60 秒記録 | `python dmmlog.py stream COM5 "0 com" --seconds 60 -o ain0.csv` |
| 2つの基準の差を 1 分ごとに 2 時間 | `python dmmlog.py repeat COM5 "refdiff 20" --every 60 --count 120 -o refdiff.csv` |
| 電源と温度を 10 秒ごとに | `python dmmlog.py repeat COM5 "mon" --every 10 --count 60 -o mon.csv` |
| グラフにする(電圧) | `python dmmlog.py plot ain0.csv` |
| グラフにする(小さい差) | `python dmmlog.py plot refdiff.csv --column uV --title "ADR4525 − 内蔵の基準"` |

- ポート名(`COM5` など)は、Arduino IDE の「ツール → ポート」かデバイスマネージャで見る。
- ポートの代わりに `fake` と書くと、機器なしで動きを試せる(それらしい作り物の値が出る)。
- グラフは CSV と同じ名前の `.png`。左が時間ごとの「平均からのずれ」、右がその分布(ヒストグラム)と標準偏差 σ。
- `Ctrl+C` で途中で止めても、そこまでの分は保存される。
- `mon` の CSV は AVDD・DVDD・温度が1列(`f2`)に混ざる。温度だけ見るときは表計算ソフトで `f1` が `TEMP_C` の行を選ぶ。

記事の図にするときは、`--title` に何を測ったかを書いておく。
