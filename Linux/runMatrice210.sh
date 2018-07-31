#!/usr/bin/env bash
# This file must be in build/bin
# Create log directory if it doesn't exist
mkdir -p log
# Rename last log, delete _ char at the beginning
rename 's/_//g' log/*
# Launch program and store in parallel the console output
# Log are formatting as follow : log[index]-[yyyy][mm][dd]-[hh][mm][ss].log
# GMT Date/Time is used 
# Index is incremented to have numbered log
# Last log start with _ char
./matrice210 0 | tee log/_log`ls log/log* | wc -l`-$(date +%Y%m%d-%H%M%S).log
# Current log file can be readed in real time with command : tail -f _*.log
