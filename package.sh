#!/bin/bash
rm polylidar.*pyd
rm polylidar.*so

rm -rf build
rm -rf dist

nox

# Upload Notes - Jeremy
# Do from WSL on Windows
# twine upload -r testpypi dist/*
# twine upload dist/*