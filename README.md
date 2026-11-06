# scenario_navigation  
人が道案内に用いる情報（シナリオ）を分解し,「条件」と「行動」を抽出するパッケージ

## 実行状況の可視化
`cmd_dir_executor_detailed`は`/scenario_status`（`std_msgs/String`、JSON）に現在の状態を出力します。次のコマンドで、シナリオの進み具合、現在の通路クラス、受理するラベル、旋回距離、CAREの作動を画像で表示できます。
```bash
roslaunch scenario_navigation scenario_overlay.launch
```
画像は`/scenario_navigation/overlay`に出力します。`/scenario_status`が1秒以上届かないと`STALE`を表示します。

## パッケージの構成
```
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config
│   └── Scenarios
│       ├── images_scenario01-50.pdf
│       ├── scenario01.txt
│       ├── scenario02.txt
│       ├── scenario03.txt
│       ├── scenario04.txt
#################### サンプルのシナリオ ###########################
│       └── scenario50.txt
├── launch
│   └── scenario_navigation.launch
├── msg
│   └── PassageType.msg
├── package.xml
├── scripts
│   └── scenario_parser_jsk.py
├── src
│   └── cmd_dir_executor_detailed.cpp
└── srv
    └── Scenario.srv
```




