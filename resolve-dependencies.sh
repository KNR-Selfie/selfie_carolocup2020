#!/usr/bin/env bash

git submodule init
git submodule update
rosdep install --from-paths src --ignore-src -r -y
