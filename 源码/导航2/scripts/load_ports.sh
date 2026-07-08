#!/usr/bin/env bash
# 被 source 时只提供 load_device_ports 函数，不改父脚本的 set -u
_LOAD_PORTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_LOAD_PORTS_PY="${_LOAD_PORTS_DIR}/load_ports.py"

load_device_ports() {
  # shellcheck disable=SC2046
  eval "$(python3 "${_LOAD_PORTS_PY}" --bash "$@")"
}
