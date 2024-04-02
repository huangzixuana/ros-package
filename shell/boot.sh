#!/bin/bash


export PATH_SHELL=$(dirname $(readlink -f $0))
export FILE_LOG=$(dirname $(readlink -f $0))/file.log


$PATH_SHELL/pip.sh python3-package n | tee -a $FILE_LOG;

exit 0