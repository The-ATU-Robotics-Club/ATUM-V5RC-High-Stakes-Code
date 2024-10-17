#!/bin/bash

if ! grep -Fxq "$1" brain-id.txt
then
    echo $1 > brain-id.txt
    rm bin/main.cpp.o
fi