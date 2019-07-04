#!/bin/bash

: ${NANOPB_PATH:=.}

cd ${NANOPB_PATH}
echo "Creating C nanopb files..."
protoc --plugin=protoc-gen-nanopb=./generator/protoc-gen-nanopb --nanopb_out=../Src mnist_predict.proto

echo "Creating Python protobuffer files"
protoc --python_out=python mnist_predict.proto

cd -
