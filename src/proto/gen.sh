#!/bin/bash

shopt -s globstar

rm -rf generated
mkdir -p generated
protoc -Iros-message-schemas/proto --cpp_out=generated ros-message-schemas/proto/**/*.proto
protoc -I. -Iros-message-schemas/proto --cpp_out=generated zoomies/**/*.proto
