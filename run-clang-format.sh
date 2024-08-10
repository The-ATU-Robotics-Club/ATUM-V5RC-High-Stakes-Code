#!/bin/bash
shopt -s globstar
clang-format include/atum/api/**/*.h include/atum/impl/**/*.h src/**/*.c -i -style=file
clang-format include/atum/api/**/*.hpp include/atum/impl/**/*.hpp src/**/*.cpp -i -style=file
clang-format test/api/**/*.hpp test/api/**/*.c test/api/**/*.cpp -i test/config.hpp test/main.cpp -style=file