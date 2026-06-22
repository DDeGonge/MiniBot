#!/bin/bash

# Format all files recursively in the firmware directory
# Supports Python (.py) and C++ (.cpp, .h) files

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "Formatting Python files with black..."
echo "=========================================="
if command -v black &> /dev/null; then
    find "$SCRIPT_DIR" -name "*.py" -type f | while read file; do
        echo "Formatting: $file"
        black "$file"
    done
else
    echo "⚠️  black not found. Install with: pip install black"
fi

echo ""
echo "=========================================="
echo "Formatting C++ files with clang-format..."
echo "=========================================="
if command -v clang-format &> /dev/null; then
    find "$SCRIPT_DIR" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) | while read file; do
        echo "Formatting: $file"
        clang-format -i "$file"
    done
else
    echo "⚠️  clang-format not found. Install with:"
    echo "   Ubuntu/Debian: sudo apt-get install clang-format"
    echo "   macOS: brew install clang-format"
    echo "   Windows: winget install llvm"
fi

echo ""
echo "=========================================="
echo "✅ Formatting complete!"
echo "=========================================="
