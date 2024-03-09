
loopian::ORBIT in Rust
===================================

loopian::ORBIT とは
------------------------

触るだけで、今流れている音楽に最適な音階を奏でてくれる円弧状の電子楽器。

仕様
--------------

Setup Mode
- Joystick を押しながら電源をONにすると、"Su" が表示され、セットアップモードに入る
- このモードでは、Sensor基板の中の CY8CMBR3110 に設定情報を書き込む
    - Joystick を左右に操作し、書き込みたい番号を選択、真ん中のスイッチを押す
- 書き込みが成功した場合、ErrLEDが3回点滅し、"Ok" が表示される
    - 失敗した場合 "Err" が表示される
    - デフォルトのI2Cアドレスのボードがなかった場合、指示されたアドレスがない場合、書き込み対象が無いので、"--"が表示される
- LED Check をする場合、Joystick を左右に操作した際、「LE」を選択、真ん中のスイッチを押す
    - 隣り合うLEDが交互に光るパターンが現れる

通常立ち上げ
- すでにアドレスが存在した場合、あるいは正常に書き込みできた場合 "00","01","02",・・・,"11" の表示が現れる
- 12種類の全てのボードの存在が確認できた場合 "Ok" と表示され、最後に12個のドットが表示される
- 1種類でもボードが足りない場合、"ま" と表示され、その後、最後に確認に成功したボードのドットが表示される
