#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

rmmod rpigpio-rec
insmod rpigpio-rec.ko gpios=4,5 num_gpios=2
./install-rpigpio-rec.sh 2

