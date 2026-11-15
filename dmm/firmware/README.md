# DMM のファーム

## dmm_stageA(Arduino Mega、試作 A)

- 配線は `../stageA.md`。レジスタとコマンドの値はデータシート SBAS660C から(コードのコメントに表の番号)。
- 確認: `arduino-cli compile -b arduino:avr:mega`(`USE_OLED 0`)で警告なしに通る(2026-11-10)。**OLED を使う版(`USE_OLED 1`)はまだビルドしていない。**
- OLED を使うときは、Arduino IDE のライブラリマネージャで **Adafruit SSD1306** を入れる(Adafruit GFX と BusIO も一緒に入る)。まだ OLED が無ければ、先頭の `USE_OLED` を 0 にする。

### 使い方

シリアルモニタを 115200 baud、改行コード「LF」にして、1行に1つコマンドを送る。

| コマンド | 内容 |
|---|---|
| `id` | ID レジスタ。下位 3 bit が 000 なら ADS124S08。**最初にこれが通るか確かめる** |
| `regs` | レジスタ 00h〜11h を全部出す |
| `mon` | AVDD・DVDD・チップの温度(内部の監視回路) |
| `ref int` / `ref ext` | 物差し: 内蔵の 2.5 V / REFP0 の ADR4525 |
| `rate 0..13` | データレート(4 = 20 SPS が既定。50 Hz と 60 Hz を両方落とす) |
| `v 0 com` | AIN0 − AINCOM(= GND)を測る。`v 0 1 8 50` は AIN0 − AIN1、利得 8、50 回の平均 |
| `refdiff` | ADR4525 − 内蔵の基準 を利得 128 で。保証の範囲(1.75 mV)に入るかも出す |
| `noise 100` | ADC の中で入力を短絡して 100 回。オフセットとノイズ(標準偏差) |
| `stream 0 com` | 1回ごとに1行出し続ける(何か送ると止まる)。PC で記録して波形にする用 |

出力は `v,AIN0-COM,gain=1,ref=int,n=10,code=…,V=…,sd_uV=…` のようにカンマ区切りで、PC でそのまま CSV として読める。

### 作りの決まり

- 変換は1回ずつ(シングルショット): START コマンド → DRDY が下がるのを待つ → RDATA。入力を切り替えても毎回落ち着いた値になる。
- 利得 1 は PGA を通さない(入力が GND の近くでも測れる)。利得 2 以上は PGA を通す(入力は AVSS + 0.15 V 〜 AVDD − 0.15 V の内側。利得 32 以上はさらに狭い)。
- AVR の `double` は 32 bit なので、平均は整数で足してから割る。表示の 7 桁目は当てにしない。
