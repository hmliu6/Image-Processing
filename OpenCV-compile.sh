

#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Input cpp filename for compilation in first argument"
    exit 0
fi

g++ "${1}.cpp" -o "${1}.o" `pkg-config --cflags --libs /usr/local/Cellar/opencv/3.4.0/lib/pkgconfig/opencv.pc` && ./"${1}.o"

rm "${1}.o"