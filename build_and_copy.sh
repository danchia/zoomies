set -e 
cmake --build . -t driver -j
scp driver/driver pi@rpi:/home/pi/car/driver
#scp hw/camera_test pi@rpi:/home/pi/car/camera_test
