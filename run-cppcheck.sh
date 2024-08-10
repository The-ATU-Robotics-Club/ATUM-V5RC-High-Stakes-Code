#!/bin/bash
cppcheck --enable=all -DDISABLE_PREDEFINED_UNITS -I include/atum/**/* src test/api 