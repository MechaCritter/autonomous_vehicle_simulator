#!/bin/bash
# compile_proto.sh - Compiles sensor_data.proto into C++ source and header files.

# Set directories - adjust these paths if your project structure changes.
PROTO_DIR="./sensors/proto"
OUT_DIR="$PROTO_DIR/generated"
PROTO_FILE="sensor_data.proto"

# Create the output directory if it doesn't exist.
mkdir -p "$OUT_DIR"

# Locate the grpc_cpp_plugin executable.
GRPC_CPP_PLUGIN_PATH=$(which grpc_cpp_plugin)
if [ -z "$GRPC_CPP_PLUGIN_PATH" ]; then
    echo "Error: grpc_cpp_plugin not found in your PATH."
    exit 1
fi

echo "Compiling $PROTO_FILE into C++ sources..."

# Run the protocol buffer compiler for both C++ and gRPC code generation.
protoc -I="$PROTO_DIR" \
    --cpp_out="$OUT_DIR" \
    --grpc_out="$OUT_DIR" \
    --plugin=protoc-gen-grpc="$GRPC_CPP_PLUGIN_PATH" \
    "$PROTO_DIR/$PROTO_FILE"

if [ $? -eq 0 ]; then
    echo "Compilation successful. Output files are located in $OUT_DIR."
else
    echo "Compilation failed."
    exit 1
fi