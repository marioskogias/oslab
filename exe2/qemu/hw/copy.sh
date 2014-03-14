#!/bin/bash

cp *  ~/qemu-1.2.0/hw
cur_dir=$(pwd)
cd ~/qemu-1.2.0/
make 
make install
cd $cur_dir
