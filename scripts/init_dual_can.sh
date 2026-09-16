#!/usr/bin/env bash

set -Eeuo pipefail

LEFT_CAN_NAME="${LEFT_CAN_NAME:-can_piper_left}"
RIGHT_CAN_NAME="${RIGHT_CAN_NAME:-can_piper_right}"

# 来自旧 piper 仓库的实际 USB 物理端口映射
LEFT_USB_BUS="${LEFT_USB_BUS:-1-2:1.0}"
RIGHT_USB_BUS="${RIGHT_USB_BUS:-1-1:1.0}"

CAN_BITRATE="${CAN_BITRATE:-1000000}"

log()
{
    echo "[dual-can-init] $*"
}

die()
{
    echo "[dual-can-init][ERROR] $*" >&2
    exit 1
}

command -v ip >/dev/null || die "ip command not found"
command -v ethtool >/dev/null || die "ethtool not found"
command -v candump >/dev/null || die "can-utils/candump not found"

find_iface_by_bus()
{
    local wanted_bus="$1"
    local iface
    local bus

    for iface in $(ip -br link show type can | awk '{print $1}'); do
        bus="$(
            sudo ethtool -i "${iface}" 2>/dev/null |
            awk '/bus-info:/ {print $2}'
        )"

        if [[ "${bus}" == "${wanted_bus}" ]]; then
            echo "${iface}"
            return 0
        fi
    done

    return 1
}

show_can_devices()
{
    local iface
    local bus

    log "Current CAN devices:"

    for iface in $(ip -br link show type can | awk '{print $1}'); do
        bus="$(
            sudo ethtool -i "${iface}" 2>/dev/null |
            awk '/bus-info:/ {print $2}'
        )"

        echo "  ${iface} -> ${bus:-unknown}"
    done
}

activate_can()
{
    local usb_bus="$1"
    local target_name="$2"

    local iface
    iface="$(find_iface_by_bus "${usb_bus}")" ||
        die "No CAN adapter found on USB bus ${usb_bus}"

    log "${usb_bus}: found interface ${iface}"

    if [[ "${iface}" != "${target_name}" ]]; then

        # 防止名字被另一块设备占用
        if ip link show "${target_name}" >/dev/null 2>&1; then
            local existing_bus
            existing_bus="$(
                sudo ethtool -i "${target_name}" 2>/dev/null |
                awk '/bus-info:/ {print $2}'
            )"

            if [[ "${existing_bus}" != "${usb_bus}" ]]; then
                die "${target_name} already exists on USB bus ${existing_bus}"
            fi
        else
            log "Renaming ${iface} -> ${target_name}"

            sudo ip link set "${iface}" down
            sudo ip link set "${iface}" name "${target_name}"
        fi
    fi

    log "Configuring ${target_name} at ${CAN_BITRATE} bit/s"

    sudo ip link set "${target_name}" down || true
    sudo ip link set "${target_name}" type can bitrate "${CAN_BITRATE}"
    sudo ip link set "${target_name}" up

    log "${target_name} activated"
}

show_can_devices

# 与旧工程保持相同顺序：右 → 左
activate_can "${RIGHT_USB_BUS}" "${RIGHT_CAN_NAME}"
activate_can "${LEFT_USB_BUS}" "${LEFT_CAN_NAME}"

echo
log "Final configuration:"

ip -details link show "${LEFT_CAN_NAME}"
echo
ip -details link show "${RIGHT_CAN_NAME}"

echo
log "CAN initialization complete."