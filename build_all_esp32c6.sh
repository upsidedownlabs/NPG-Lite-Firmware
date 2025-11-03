#!/bin/bash

# Configuration
BOARD_FQBN="esp32:esp32:esp32c6"
OUTPUT_DIR="compiled_binaries"
LIBRARY_DIR="$HOME/Arduino/libraries"

# Install dependencies
echo "➡️ Installing dependencies..."
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.2.0
arduino-cli lib install "BLE"

# Create fresh output directory
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# Compilation function with error handling
compile_project() {
  local sketch_dir=$1
  local sketch_name=$(basename "$sketch_dir")
  local output_path="$OUTPUT_DIR/$sketch_name"

  mkdir -p "$output_path"

  echo -e "\n🛠️  Compiling: $sketch_name"
  if arduino-cli compile \
    --fqbn "$BOARD_FQBN" \
    --output-dir "$output_path" \
    --libraries "$LIBRARY_DIR" \
    --export-binaries \
    --warnings default \
    "$sketch_dir" > "$output_path/compile.log" 2>&1; then

    echo "✅ Success"
    echo "   Binary: $output_path/$sketch_name.ino.bin"
  else
    echo "❌ Failed"
    echo "   Compilation log:"
    echo "   ========================================="
    cat "$output_path/compile.log"
    echo "   ========================================="
    echo "   Full log also saved to: $output_path/compile.log"
  fi
}

# Process all projects
find . -maxdepth 1 -type d -name "[!.]*" | while read sketch_dir; do
  if [ -f "$sketch_dir/$(basename "$sketch_dir").ino" ]; then
    compile_project "$sketch_dir"
  else
    echo "⚠️  Skipping non-Arduino folder: $(basename "$sketch_dir")"
  fi
done

echo -e "\n📊 Compilation summary:"
find "$OUTPUT_DIR" -name "*.bin" -exec ls -lh {} \;