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

## dmm_stageB(Arduino Mega、試作 B)

- 配線は `../stageB.md` と `../stageB.schjson`、組む順は `../stageB-build.md`。
- 確認: `arduino-cli compile -b arduino:avr:mega --warnings all`(`USE_OLED 0`)で、スケッチの警告なしに通る(2026-11-16。フラッシュ 23 KB・RAM 715 バイト)。**実機ではまだ動かしていない。**
- A との違い: 測るのはいつも VCOM(AIN3、LO 端子)との差。AINCOM は使わない。基準は起動時から ADR4525(`ref ext`)。`refdiff` は AIN8 − AIN9。
- 起動のたびに、K1〜K8 を全部リセットする(ラッチリレーは電源を切っても前の状態のまま)。コイルは 1 個ずつ、10 ms のパルス。
- 校正の値は EEPROM(先頭の 124 バイト)。`cal save` するまで書かない。

### 使い方(A のコマンドに加えて)

| コマンド | 内容 |
|---|---|
| `mode v100m` / `v1` / `v10` / `v30` | 電圧のレンジ(満量程 ±156 mV / ±1.25 V / ±20.2 V / ±33.3 V) |
| `mode r100` … `r10m` `[2w\|4w]` | 抵抗のレンジ(基準抵抗 1 kΩ〜10 MΩ。r100 と r1k は同じ K1)。**抵抗のモードで電圧をかけない** |
| `meas [count]` | いまのモードで 1 回測る。抵抗は V(Rx) → V(Rref) → V(Rx) の順(A-B-A)で、`count` 回ずつ平均。範囲を超えたら `value=OL` |
| `relay` / `relay init` / `relay <1..8> set\|reset` / `relay pulse <ms>` | リレーの状態(S = セット、r = リセット、K1 から)/ 全部リセット / 1 個だけ動かす(手で動かしたあとは `mode` をやり直すまで `meas` しない)/ パルスの長さ |
| `settle <ms>` | リレーを動かしてから測るまでの待ち(既定 100 ms) |
| `cal` | 校正の値の一覧 |
| `cal zero [count]` | いまのモードで入力を短絡して、ゼロを取る(電圧はレンジごと、抵抗はレンジと 2 線 / 4 線ごと) |
| `cal open [count]` | 抵抗のモードで入力を開いて、入力をまたぐ漏れを抵抗として取る(r1m・r10m 用。見えなければ `none`) |
| `cal ratio a` / `cal ratio b` | 分圧の比。a は約 1 V を入れて 1 V と 10 V のレンジで、b は約 15 V を入れて 10 V と 30 V のレンジで読む |
| `cal rref <mode> <ohms>` | 基準抵抗の値(外で校正した値) |
| `cal save` / `cal load` / `cal clear` | EEPROM に保存 / 読み直す / 設計の値に戻す(保存はしない) |

出力の例: `meas,mode=r10k,wire=4w,n=10,value=9998.1234,unit=ohm,sd=0.0123,vx_V=1.0249,vref_V=1.0251`。`tools/dmmlog.py repeat COM5 "meas 10" …` でそのまま CSV に記録できる。

### 計算

- 電圧: `(ADC の読み − ゼロ) × 比`。比は 1(v100m・v1)、`ratioA`(v10、設計 16.15)、`ratioB`(v30、設計 106.6)。
- 抵抗: `Rx = Rref × V(Rx) / V(Rref)`。開放の校正があれば `1/Rx − 1/ropen` で漏れを除き、そのあと短絡のゼロを引く。開いた接点の漏れのうち AVDD 側(Rref と並列)のものは、開放の校正では見えない(`../stageB.md` の見直しの 6)。
