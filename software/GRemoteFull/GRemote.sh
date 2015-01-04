#!/bin/bash

JAVA_BIN=`which java`
if [ $? -ne 0 ]; then
   echo "Please install java to run GRemote"
   exit 1
fi

$JAVA_BIN -Djava.ext.dirs=lib -Djava.library.path=lib GRemote
