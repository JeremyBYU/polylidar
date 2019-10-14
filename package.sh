#!/bin/bash
rm polylidar.*pyd
rm polylidar.*so

rm -rf build
rm -rf dist

nox
