#!/bin/bash
# Calculate the number of threads on the host machine and set THREADS environment variable

# Get the number of threads
THREADS=$(grep -c ^processor /proc/cpuinfo)

# Print the result
echo $THREADS
