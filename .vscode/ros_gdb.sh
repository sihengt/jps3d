#!/bin/bash
source /home/cf_drone/install/setup.bash
exec /usr/bin/gdb "$@"
