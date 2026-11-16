#!/usr/bin/env bash
# =============================================================================
# setup-home-assistant.sh - Home Assistant Container (Docker) on an Ubuntu server
#
# Designed to be SAFE while you are logged in over SSH:
#   * fully non-interactive: never reads stdin, never asks a question, never uses a pager
#   * every long job is bounded by `timeout`, so a stuck download cannot freeze your session
#   * the container is ALWAYS started detached (-d / compose up -d): nothing holds your tty
#   * never restarts sshd, edits ssh config/keys, enables ufw, or reboots the server
#   * never runs dist-upgrade / do-release-upgrade (those can restart sshd and drop you)
#   * never uses pkill/killall on host processes - only `docker stop`/`docker rm` on its own
#     container name
#   * run it, never `source` it, so its `exit` cannot log you out
#
# Recommended way to run it over SSH (survives a dropped connection):
#   from Windows:  scp scripts/setup-home-assistant.sh user@server:~/
#   ssh -t user@server
#   sudo -v                     # cache sudo credentials so the background job cannot prompt
#   sudo sh -c 'nohup bash ~/setup-home-assistant.sh < /dev/null >/tmp/ha-setup.log 2>&1 &'
#   tail -n 50 /tmp/ha-setup.log          # read the log AFTER, never `tail -f` over ssh
#
# Usage: sudo bash ./setup-home-assistant.sh [options]
#   --name NAME            container name                 (default homeassistant)
#   --data-dir DIR         base dir for config/share      (default /srv/homeassistant)
#   --image REF            container image                (default ghcr.io/home-assistant/core:stable)
#   --tz NAME              IANA timezone name             (default auto-detect -> Etc/UTC)
#   --port N               port HA listens on (checks/reporting only) (default 8123)
#   --owner UID:GID        chown the data dirs to this owner, e.g. 1000:1000
#   --device DEV           bind a host device, repeatable, e.g. /dev/ttyUSB0
#   --dbus yes|no|auto     bind-mount /run/dbus read-only (default auto: mount if present)
#   --compose              start with `docker compose` (compose.yaml is written either way)
#   --skip-docker          assume Docker Engine is already installed
#   --official-script      install Docker from get.docker.com instead of distro packages
#   --recreate             stop + remove an existing container before starting a new one
#   --dry-run              print the commands instead of running them
#   -h, --help             this help
#
# Home Assistant Container needs Docker Engine >= 23.0.0 (Docker Desktop is irrelevant here).
# Note: Home Assistant Container has no "apps" and no system control; that is the documented
# limitation of the Container installation type.
# =============================================================================
set -Eeuo pipefail

# ------------------------------- defaults ----------------------------------
CONTAINER="homeassistant"
DATA_DIR="/srv/homeassistant"
IMAGE="ghcr.io/home-assistant/core:stable"
HA_PORT=8123
TZ_NAME="auto"
OWNER=""
DBUS="auto"
USE_COMPOSE=0
SKIP_DOCKER=0
DOCKER_OFFICIAL=0
RECREATE=0
DRY_RUN=0
DEVICE_ARGS=()
APT_TIMEOUT=900        # seconds: apt never waits forever on a cache lock
PULL_TIMEOUT=1800      # seconds: docker pull never waits forever
VERIFY_SECONDS=90      # seconds: bounded loop waiting for HA to answer on HA_PORT

# ------------------------------- helpers ----------------------------------
say()  { printf '%s\n' "$*"; }
note() { printf '[*] %s\n' "$*"; }
warn() { printf '[!] %s\n' "$*" >&2; }
have() { command -v "$1" >/dev/null 2>&1; }
die()  { printf '[x] %s\n' "$*"; exit 1; }

on_exit() {
  local rc=$?
  if [ "$rc" -ne 0 ]; then
    warn "aborted with exit code ${rc} at command: ${BASH_COMMAND:-unknown}"
  fi
}
trap on_exit EXIT

step() {
  # step <description> <command...>  -> logs then runs; honours --dry-run
  local desc="$1"
  shift
  if [ "$DRY_RUN" -eq 1 ]; then
    say "[dry-run] ${desc} :: $*"
    return 0
  fi
  say "[run] ${desc}"
  "$@"
}

usage() {
  sed -n '2,41p' "$0" | sed 's/^# \{0,1\}//'
}

# ---------------------------- option parsing ------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --name)            CONTAINER="${2:?--name needs a value}"; shift 2 ;;
    --data-dir)        DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
    --image)           IMAGE="${2:?--image needs a value}"; shift 2 ;;
    --tz)              TZ_NAME="${2:?--tz needs a value}"; shift 2 ;;
    --port)            HA_PORT="${2:?--port needs a value}"; shift 2 ;;
    --owner)           OWNER="${2:?--owner needs a value}"; shift 2 ;;
    --dbus)            DBUS="${2:?--dbus needs a value}"; shift 2 ;;
    --device)          d="${2:?--device needs a value}"; DEVICE_ARGS+=("--device" "${d}:${d}"); shift 2 ;;
    --compose)         USE_COMPOSE=1; shift ;;
    --skip-docker)     SKIP_DOCKER=1; shift ;;
    --official-script) DOCKER_OFFICIAL=1; shift ;;
    --recreate)        RECREATE=1; shift ;;
    --dry-run)         DRY_RUN=1; shift ;;
    -h|--help)         usage; exit 0 ;;
    *)                 die "unknown option: $1 (see --help)" ;;
  esac
done

# -------------------- timezone auto-detection (no prompts) -----------------
if [ "$TZ_NAME" = "auto" ]; then
  if have timedatectl; then
    TZ_NAME="$(timedatectl show --property=Timezone --value 2>/dev/null || true)"
  fi
  if [ -z "${TZ_NAME:-}" ] && [ -e /etc/localtime ]; then
    TZ_NAME="$(readlink -f /etc/localtime 2>/dev/null | sed -e 's#.*/zoneinfo/##' -e 's#.*/locallow/##')"
  fi
  case "${TZ_NAME:-}" in
    ''|auto) TZ_NAME="Etc/UTC" ;;
  esac
fi
note "container timezone will be TZ=${TZ_NAME} (pass --tz to override)"

# ------------------------- 1. preflight (no prompts) ----------------------
if [ "$EUID" -ne 0 ]; then
  # Re-run under sudo only if we can do it WITHOUT blocking on a password prompt:
  # either stdin is a real tty (ssh -t) or the sudo credentials are already cached.
  if have sudo; then
    if sudo -n true 2>/dev/null; then
      say "[+] re-executing with sudo (credentials already cached)"
      exec sudo -E bash "$0" "$@"
    elif [ -t 0 ]; then
      say "[+] re-executing with sudo; a password prompt is safe because stdin is a tty"
      exec sudo -E bash "$0" "$@"
    else
      warn "needs root, and stdin is not a tty - so it cannot prompt (it will never block either)."
      say "  Do this instead: ssh -t user@server, run 'sudo -v' first to cache credentials,"
      say "  then: sudo sh -c 'nohup bash ~/setup-home-assistant.sh < /dev/null >/tmp/ha-setup.log 2>&1 &'"
      exit 1
    fi
  else
    die "no sudo found: run this script as root (bash ./setup-home-assistant.sh)"
  fi
fi

OS_RELEASE="$(cat /etc/os-release 2>/dev/null || true)"
if printf '%s' "$OS_RELEASE" | grep -qiE 'ubuntu|debian'; then
  note "distribution looks like Debian/Ubuntu -> apt install path enabled"
  NONAPT=0
else
  warn "not Debian/Ubuntu: apt install steps are skipped, everything else still runs"
  NONAPT=1
fi

for c in curl sed grep; do
  have "$c" || die "missing required core utility: $c"
done

free_kb="$(df -Pk / 2>/dev/null | tail -1 | awk '{print $4}')"
if [ -n "${free_kb:-}" ] && [ "$free_kb" -lt 1048576 ]; then
  warn "less than 1 GB free on / - Home Assistant plus its image need the space"
fi

# ---------------------- 2. Docker Engine (bounded, quiet) -----------------
if [ "$SKIP_DOCKER" -eq 0 ] && ! have docker; then
  if [ "$NONAPT" -eq 1 ]; then
    die "Docker is not installed and apt is unavailable - install Docker Engine first"
  fi
  note "installing Docker Engine. This does not touch sshd and performs no kernel/upgrade work."
  if [ "$DOCKER_OFFICIAL" -eq 1 ]; then
    step "download get.docker.com" curl -fsSL --max-time 30 -o /tmp/get-docker.sh https://get.docker.com
    step "run get-docker.sh" bash /tmp/get-docker.sh
  else
    installed=0
    for pkg in docker-server docker.io; do
      if step "apt install $pkg" timeout "$APT_TIMEOUT" env \
          DEBIAN_FRONTEND=noninteractive APT_SKIP_CONFIRMATIONS=1 \
          apt-get -y --no-progress install "$pkg"; then
        installed=1
        break
      fi
      note "apt package '$pkg' not available on this release, trying the next candidate"
    done
    if [ "$installed" -eq 0 ]; then
      die "no distro Docker package matched - re-run with --official-script (get.docker.com)"
    fi
  fi
  have docker || die "Docker CLI is still missing after install"
fi

if have docker && [ "$USE_COMPOSE" -eq 1 ] && ! docker compose version >/dev/null 2>&1; then
  if [ "$NONAPT" -eq 0 ]; then
    step "apt install docker-compose-plugin" timeout "$APT_TIMEOUT" env \
        DEBIAN_FRONTEND=noninteractive APT_SKIP_CONFIRMATIONS=1 \
        apt-get -y --no-progress install docker-compose-plugin
  else
    warn "docker compose unavailable; falling back to plain docker run"
    USE_COMPOSE=0
  fi
fi

# ------------------- 3. docker daemon on/off (systemd only) --------------
if have docker && have systemctl; then
  if systemctl is-active --quiet docker; then
    note "docker daemon already running"
  else
    step "systemctl start docker" systemctl start docker
  fi
  systemctl is-enabled --quiet docker || step "systemctl enable docker" systemctl enable docker
fi

# --------------------------- 4. data directories -------------------------
step "create ${DATA_DIR}/config and ${DATA_DIR}/share" \
  install -d -m 0755 "${DATA_DIR}/config" "${DATA_DIR}/share"

if [ -n "$OWNER" ]; then
  step "chown ${DATA_DIR} -> ${OWNER}" chown -R "$OWNER" "$DATA_DIR"
fi

if [ "$HA_PORT" != "8123" ]; then
  note "Home Assistant itself listens on 8123 by default; --port only changes the check/report."
fi
if ss -ltn 2>/dev/null | grep -Eq ":${HA_PORT}([^0-9]|$)"; then
  warn "something already listens on :${HA_PORT} - Home Assistant may not be reachable"
fi

# ----------------------- 5. compose.yaml (for updates) -------------------
COMPOSE_FILE="${DATA_DIR}/compose.yaml"

if [ -e /run/dbus ] && [ "$DBUS" != "no" ]; then
  DBUS_MOUNT=1
elif [ "$DBUS" = "yes" ]; then
  DBUS_MOUNT=1
else
  DBUS_MOUNT=0
  note "no read-only /run/dbus bind (only needed for the Bluetooth integration)"
fi

if [ "$DRY_RUN" -eq 0 ]; then
  {
    printf '%s\n' \
      "# Home Assistant Container - generated by setup-home-assistant.sh" \
      "# start:  docker compose -f ${COMPOSE_FILE} up -d" \
      "# stop:   docker compose -f ${COMPOSE_FILE} stop" \
      "# logs:   docker compose -f ${COMPOSE_FILE} logs --tail 50   (never add -f over ssh)" \
      "services:" \
      "  homeassistant:" \
      "    container_name: ${CONTAINER}" \
      "    image: ${IMAGE}" \
      "    volumes:" \
      "      - ${DATA_DIR}/config:/config" \
      "      - ${DATA_DIR}/share:/share" \
      "      - /etc/localtime:/etc/localtime:ro"
    if [ "$DBUS_MOUNT" -eq 1 ]; then
      printf '%s\n' "      - /run/dbus:/run/dbus:ro"
    fi
    if [ "${#DEVICE_ARGS[@]}" -gt 0 ]; then
      printf '%s\n' "    devices:"
      for d in "${DEVICE_ARGS[@]}"; do
        if [ "$d" = "--device" ]; then continue; fi
        printf '%s\n' "      - ${d}"
      done
    fi
    printf '%s\n' \
      "    restart: unless-stopped" \
      "    stop_grace_period: 60s" \
      "    privileged: true" \
      "    network_mode: host" \
      "    environment:" \
      "      TZ: ${TZ_NAME}"
  } > "$COMPOSE_FILE"
  note "wrote ${COMPOSE_FILE}"
else
  note "dry-run: ${COMPOSE_FILE} would be written"
fi

# ---------------- 6. existing container (idempotent handling) ------------
existing=0
if have docker; then
  if docker ps -a --format '{{.Name}}' 2>/dev/null | grep -qx "^/${CONTAINER}\$" \
    || docker ps -a --format '{{.Name}}' 2>/dev/null | grep -qx "^${CONTAINER}\$"; then
    existing=1
  fi
fi
if [ "$existing" -eq 1 ]; then
  if [ "$RECREATE" -eq 1 ]; then
    step "stop existing container" docker stop "$CONTAINER"
    step "remove existing container" docker rm "$CONTAINER"
  else
    note "container '${CONTAINER}' already exists - leaving it alone (use --recreate to replace it)"
  fi
fi

# ------------------- 7. pull + start (always detached) -------------------
if [ "$existing" -eq 1 ] && [ "$RECREATE" -eq 0 ]; then
  note "skipping pull/start because the container is already there"
else
  if ! have docker; then
    warn "docker is not available - stopping before the container start"
  else
    step "pull ${IMAGE}" timeout "$PULL_TIMEOUT" docker pull --quiet "$IMAGE"

    if [ "$USE_COMPOSE" -eq 1 ]; then
      step "docker compose up (detached)" \
        timeout 300 docker compose -f "$COMPOSE_FILE" up -d
    else
      declare -a RUN=(docker run -d --name "$CONTAINER" \
        --restart unless-stopped --stop-timeout 60 --network host --privileged \
        -e "TZ=${TZ_NAME}" \
        -v "${DATA_DIR}/config:/config" -v "${DATA_DIR}/share:/share" \
        -v /etc/localtime:/etc/localtime:ro)
      if [ "$DBUS_MOUNT" -eq 1 ]; then
        RUN+=(-v /run/dbus:/run/dbus:ro)
      fi
      if [ "${#DEVICE_ARGS[@]}" -gt 0 ]; then
        RUN+=("${DEVICE_ARGS[@]}")
      fi
      step "start ${CONTAINER} (detached)" "${RUN[@]}"
    fi
  fi
fi

# --------------- 8. bounded verification (no follow/pager) --------------
if [ "$DRY_RUN" -eq 1 ] || ! have docker; then
  note "dry-run or no docker: skipping the reachability check"
else
  reachable=0
  for _ in $(seq 1 "$(( VERIFY_SECONDS / 5 ))"); do
    if curl -fsS --max-time 5 -o /dev/null "http://127.0.0.1:${HA_PORT}"; then
      reachable=1
      break
    fi
    sleep 5
  done
  if [ "$reachable" -eq 1 ]; then
    note "Home Assistant answered on http://127.0.0.1:${HA_PORT} - onboarding can start"
  else
    warn "no answer on :${HA_PORT} within ${VERIFY_SECONDS}s. Inspect with:"
    say "  docker ps -a --filter name=${CONTAINER} ; docker logs --tail 60 ${CONTAINER}"
    say "  journalctl -n 60 -u docker          (no -f over ssh)"
  fi
fi

if have ufw && ufw status 2>/dev/null | grep -qi '^Status: active'; then
  if ufw status 2>/dev/null | grep -q "${HA_PORT}/tcp"; then
    note "ufw already allows ${HA_PORT}/tcp"
  else
    warn "ufw is active and does not allow ${HA_PORT}/tcp - if needed run: sudo ufw allow ${HA_PORT}/tcp"
    say "    (docker exposes container ports bypassing ufw; port 22 is left untouched here)"
  fi
fi

say
say "Done. Next steps:"
say "  1. Open http://<server-ip>:${HA_PORT} (or tunnel: ssh -L ${HA_PORT}:localhost:${HA_PORT} user@server)"
say "  2. Finish onboarding in the browser."
say "  3. Update later with:  sudo docker pull ${IMAGE}; sudo docker compose -f ${COMPOSE_FILE} up -d"
say "  4. Read logs without hanging the ssh session: docker logs --tail 100 ${CONTAINER}"


