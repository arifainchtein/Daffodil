


/home/ari/.arduino15/packages/esp32/tools/mkspiffs/0.2.3/mkspiffs -c /home/ari/Data/DigitalStables/Projects/daffodil/data/ --page 256 --block 8192 --size 2686976  /tmp/out.spiffs




#/home/ari/.arduino15/packages/esp32/tools/esptool_py/4.2.1/esptool.py -cd nodemcu -cb 460800 -cp /dev/ttyUSB0 -ca 0x300000 -cf /tmp/out.spiffs





/home/ari/.arduino15/packages/esp32/tools/esptool_py/4.2.1/esptool.py --chip esp32 \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --before default_reset \
  --after hard_reset \
  write_flash 0x00290000  /tmp/out.spiffs

#esptool.py -p /dev/ttyUSB0 write_flash 0x00290000 sketch.bin 0x003fffff spiffs.bin
