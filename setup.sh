#!/bin/bash

echo "Exporting PYTHONPATH:$PWD"
export PYTHONPATH=$PYTHONPATH:$(pwd)
export PATH=$PATH:$(pwd)
fuser -k /dev/ttyACM0

