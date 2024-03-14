#!/bin/bash

# Get the save folder path as input argument
folder_path="$1"

# Clear the folder
rm -rf "$folder_path"/*
