#!/bin/bash

# Function to append a directory to the LD_LIBRARY_PATH variable
append_to_ld_library_path() {
  if [[ -z "$LD_LIBRARY_PATH" ]]; then
    export LD_LIBRARY_PATH="$1"
  else
    export LD_LIBRARY_PATH="$1:$LD_LIBRARY_PATH"
  fi
}

# Function to recursively find files with .so extension and append them to LD_LIBRARY_PATH
search_and_append_libs() {
  local dir="$1"
  local files=$(find "$dir" -type f -name "*.so")

  for file in $files; do
    append_to_ld_library_path "$(dirname "$file")"
  done
}

directory=${PWD}
search_and_append_libs "$directory"

echo "LD_LIBRARY_PATH is updated:"
echo "$LD_LIBRARY_PATH"
