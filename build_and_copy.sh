#!/bin/bash
set -e 
cmake --build . -t driver -j6
scp bin/driver pi@rpi:/home/pi/car/driver
