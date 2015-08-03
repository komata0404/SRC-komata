@rem 2.1を2.2にアップグレードするためのバッチファイルを作ってみた　[2015/7/28 永谷]
@echo off
echo STM32F4 Discovery 2層基板 ver.Taiki ドライバを2.2対応させるバッチファイル
pause
COPY /Y can.cpp ..\layer_driver\base\
COPY /Y can.hpp ..\layer_driver\base\
COPY /Y pin.cpp ..\layer_driver\board\
COPY /Y pin.cpp ..\layer_driver\board\
COPY /Y can_encoder.cpp ..\layer_driver\circuit\
COPY /Y can_encoder.hpp ..\layer_driver\circuit\
echo 2.2 upgraded >> ..\log.txt
echo 必要なファイルのコピーが完了しました。
echo .
echo /layer_driver/baseにcan.hppとcan.cppが追加されたので
echo これらをプロジェクトに追加してください。
echo そうしないとたぶんコンパイルエラーになるよ。
echo .
echo /layer_driver/circuitにcan_encoder.hppとcan_encoder.cppが追加されたので
echo これらをプロジェクトに追加してください。
echo そうするとcanエンコーダノードが使えるようになるよ！
echo .
pause