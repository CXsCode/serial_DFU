# 生成settings.hex

nrfutil settings generate --family NRF52 --application app.hex --application-version 1 --bootloader-version 1 --bl-settings-version 2 settings.hex

# 合并hex文件

mergehex --merge bootloader.hex settings.hex --output tmp.hex 
mergehex --merge tmp.hex app.hex s132_nrf52_6.1.1_softdevice.hex --output merge.hex

#删除中间文件

rm tmp.hex

# 生成升级包

nrfutil pkg generate --application app_new.hex --application-version 2 --hw-version 52 --sd-req 0xB7 --key-file priv.pem app_new.zip