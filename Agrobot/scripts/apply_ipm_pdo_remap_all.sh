#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: $0 NODE_ID [NODE_ID ...]"
  echo "example: $0 1 2 3"
  exit 1
fi

for node_id in "$@"; do
  ~/agrobot_ws/scripts/apply_ipm_pdo_remap_one.sh "$node_id"
done
