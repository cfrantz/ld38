#!/bin/bash

set -e
bazel build \
    --crosstool_top=//tools/windows:toolchain --cpu=win64 \
    --workspace_status_command tools/buildstamp/get_workspace_status \
    -c opt :volcano

VERSION=$(grep BUILD_GIT_VERSION bazel-out/volatile-status.txt | cut -f2 -d' ')

tools/windows/zip4win.py \
    --mxe tools/mxe \
    --out volcano-${VERSION}.zip \
    bazel-bin/volcano
