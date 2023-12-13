#!/bin/bash

#URL
#https://github.com/Denvi/Candle/issues/147#issuecomment-350221474

sudo apt -y install \
    g++ qt5-default qttools5-dev-tools \
    libqt5serialport5-dev libqt5opengl5-dev

#git clone https://github.com/trasz/grblControl
git clone https://github.com/Denvi/Candle.git
pushd Candle
git checkout grbl_1_1

cd src
qmake

make -j4
mkdir -p ~/.local/bin/
cp -v Candle ~/.local/bin/

popd
rm -rf Candle
