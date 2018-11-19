#!/bin/bash -xve
###上のオプション: xv: ログをとる。e: エラーで止める

sudo ./travis_prepare_dummy_files.bash                     #ダミーファイルを作る
roslaunch cpimouse_run_corridor wall_stop.launch &         #バックグラウンドでノードを立ち上げ

sleep 10  #立ち上がりを待つ

values=$(paste /dev/rtmotor_raw_{l,r}0)          #左右モータの値をファイルからとってくる
[ "$values" == "566	566" ]                   #テスト

echo 1000 1000 1000 1000 > /tmp/rtlightsensor0.tmp   #壁に近いときのセンサの値
sudo mv /tmp/rtlightsensor0.tmp /dev/rtlightsensor0  #瞬時にファイルを入れ替え
sleep 1                                         #少し間をあける
values=$(paste /dev/rtmotor_raw_{l,r}0)         #テスト
[ "$values" == "0	0" ]

sudo killall -KILL rosmaster                    #マスタを停止
wait                                            #バックグラウンドのプロセスが終わるのを待つ
echo OK
