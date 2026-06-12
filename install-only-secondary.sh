#!/bin/bash

KLIPPER_PATH="${HOME}/klipper"
INSTALL_PATH="${HOME}/klipper-toolchanger-hard"

set -eu

export LC_ALL=C

EXTRAS=(
  "tool_drop_detection.py"
  "sensorless_auto_tune.py"
  "probe_multiplexer.py"
  "heater_chamber_fan.py"
  "heater_power_distributor.py"
)

function preflight_checks {
  if [ "$EUID" -eq 0 ]; then
    echo "[PRE-CHECK] This script must not be run as root!"
    exit 1
  fi

  if sudo systemctl list-units --full -all -t service --no-legend | grep -qF 'klipper.service'; then
    printf "[PRE-CHECK] Klipper service found! Continuing...\n\n"
  else
    echo "[ERROR] Klipper service not found, please install Klipper first!"
    exit 1
  fi
}

function check_download {
  local installdirname installbasename

  installdirname="$(dirname "${INSTALL_PATH}")"
  installbasename="$(basename "${INSTALL_PATH}")"

  if [ ! -d "${INSTALL_PATH}" ]; then
    echo "[DOWNLOAD] Downloading repository..."

    if git -C "${installdirname}" clone https://github.com/Contomo/klipper-toolchanger-hard.git "${installbasename}"; then
      printf "[DOWNLOAD] Download complete!\n\n"
    else
      echo "[ERROR] Download of git repository failed!"
      exit 1
    fi
  else
    printf "[DOWNLOAD] Repository already found locally. Continuing...\n\n"
  fi
}

function link_selected_extras {
  echo "[INSTALL] Linking selected extras to Klipper..."

  local src="${INSTALL_PATH}/klipper/extras"
  local dst="${KLIPPER_PATH}/klippy/extras"
  local extra
  local missing=0

  if [ ! -d "$src" ]; then
    echo "[ERROR] Extras directory not found at: $src"
    exit 1
  fi

  mkdir -p "$dst"

  for extra in "${EXTRAS[@]}"; do
    if [ ! -f "$src/$extra" ]; then
      echo "[ERROR] Missing extra: $src/$extra"
      missing=1
      continue
    fi

    ln -sfn "$src/$extra" "$dst/$extra"
    echo " → $extra"
  done

  if [ "$missing" -ne 0 ]; then
    echo "[ERROR] One or more selected extras were missing. Klipper was not restarted."
    exit 1
  fi

  printf "\n[INSTALL] Selected extras linked.\n\n"
}

function restart_klipper {
  echo "[POST-INSTALL] Restarting Klipper..."
  sudo systemctl restart klipper
}

printf "\n=============================================\n"
echo "- Klipper selected extras install script -"
printf "=============================================\n\n"

preflight_checks
check_download
link_selected_extras
restart_klipper
