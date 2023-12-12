#!/bin/bash

M=../../../Agriculture_Machinery_Work/Media/
pushd $M

youtube-dl -f best https://www.youtube.com/watch?v=SYAFYTRXfp0
mv *SYAFYTRXfp0.mp4 cherry_1.mp4

#Extract frame every 1 sec.
ffmpeg -i cherry_1.mp4 -r 1/1 cherry_1_frame_%03d.jpg

# https://gist.github.com/loretoparisi/a9277b2eb4425809066c380fed395ab3
#Extract one frame.
#ffmpeg -i in.mp4 -vf select='between(n\,x\,y)' -vsync 0 frames%d.png

popd
