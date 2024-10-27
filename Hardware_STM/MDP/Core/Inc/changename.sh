#!/bin/bash

# Loop through all .c files in the directory
for file in *.txt; do
    # Extract the filename without the .c extension
    filename="${file%.txt}"
    
    # Copy content from .c file to .txt file
    mv "$file" "$filename.h.txt"
done
