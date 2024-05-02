#!/bin/bash

    echo "make Charge battrety Icon"
cd ../u-boot/
./scripts/pack_resource.sh ../kernel/resource.img
cp resource.img ../kernel/resource.img
echo " end make Charge battrety Icon"
