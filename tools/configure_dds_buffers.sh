#!/usr/bin/env bash
# Run on the Linux receiver: sudo bash tools/configure_dds_buffers.sh --apply
# Restart DDS processes after changing their XML profile or kernel defaults.
set -euo pipefail

buffer_bytes=268435456
config_file=/etc/sysctl.d/90-astribot-sdk-dds-buffers.conf
mode=${1:---check}
if (($# > 1)) || [[ "$mode" != --check && "$mode" != --apply ]]; then
    echo "Usage: $0 [--check|--apply]" >&2
    exit 2
fi

if [[ "$mode" == --apply ]]; then
    if ((EUID != 0)); then
        echo "Run --apply with sudo on the Linux receiver." >&2
        exit 1
    fi
    install -m 0644 /dev/stdin "$config_file" <<EOF
# Astribot SDK: 256 MiB UDP socket defaults and limits.
net.core.rmem_max=$buffer_bytes
net.core.wmem_max=$buffer_bytes
net.core.rmem_default=$buffer_bytes
net.core.wmem_default=$buffer_bytes
EOF
    sysctl -p "$config_file"
fi

failed=0
for setting in net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default; do
    value=$(sysctl -n "$setting")
    printf '%s=%s (required >= %s)\n' "$setting" "$value" "$buffer_bytes"
    if ((value < buffer_bytes)); then failed=1; fi
done
exit "$failed"
