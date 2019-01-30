#! /bin/bash

for f in $(find ./module -name '*.c' -or -name '*.cpp' -or -name '*.h' -type f)
do
    ../tools/Astyle.exe -A1 -cCs4LwK -yj -pHUf -k1W1 -z2 -M --suffix=none $f
done