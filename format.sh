#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

function format() {
  filename="$1"
  clang-format -i -style=file "${filename}"
}

for filename in `find ./Src ./Inc \( -name \*.c -o -name \*.h \)`; do
  format ${filename}
done
