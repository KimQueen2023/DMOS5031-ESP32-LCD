# 1. 打开 PowerShell，进入工程目录
cd D:\esp32\ESP32-S3-LCD-1.47B-Demo\ESP32-S3-LCD-1.47B-Test

# 2. 设置 ESP-IDF 环境变量
$env:IDF_PATH = "D:\esp32\test_esp32_dmos5031\esp-idf-v5.4\esp-idf-v5.4"
$env:IDF_TOOLS_PATH = "D:\esp32\.espressif_v54"

# 3. 加载 ESP-IDF 环境（把 idf.py 加入 PATH）
. "$env:IDF_PATH\export.ps1"

# 4. 设置目标芯片（只需一次）
idf.py set-target esp32s3

