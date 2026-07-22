#!/usr/bin/env bash
# Burst an ExoKey NSGA-II run onto a Hetzner Cloud box, then destroy it.
#
#   ./cloud/hetzner.sh price          # what the CCX line ACTUALLY costs, from the API
#   ./cloud/hetzner.sh up [type]      # create the server + install deps + push the repo
#   ./cloud/hetzner.sh run  ...       # run the optimiser on it (args passed to opt.run)
#   ./cloud/hetzner.sh burn ...       # run, fetch the results, then DELETE the box. use this.
#   ./cloud/hetzner.sh run-bg ...     # run DETACHED (survives ssh drop / local reboot)
#   ./cloud/hetzner.sh watch          # poll the detached run; fetch + DELETE when it finishes
#   ./cloud/hetzner.sh tail           # tail the detached run's log
#   ./cloud/hetzner.sh fetch          # pull out/ back
#   ./cloud/hetzner.sh down           # DELETE the server (this is what stops the billing)
#   ./cloud/hetzner.sh status         # what exists and roughly what it has cost
#
# ⚠ POWERING OFF DOES NOT STOP THE BILLING. Hetzner Cloud charges hourly for as long as the
# server EXISTS, running or not -- it is holding the vCPU/RAM/IP reservation for you. Only
# `down` (DELETE) stops the meter. There is deliberately no "stop" subcommand here, because
# a "stop" that keeps charging you is a trap, not a feature.
#
# Token: export HCLOUD_TOKEN=... , or put it in ~/.config/hcloud/token (chmod 600).
# Never paste it into a chat transcript.
set -euo pipefail

API=https://api.hetzner.cloud/v1
NAME=${EXOKEY_SERVER:-exokey-burst}
TYPE_DEFAULT=ccx53
IMAGE=ubuntu-24.04
LOCATION=${HCLOUD_LOCATION:-nbg1}
SSH_KEY_FILE=${SSH_KEY_FILE:-$HOME/.ssh/id_ed25519.pub}
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

token() {
  local f
  for f in "$HOME/hetzner.api" "$HOME/.config/hcloud/token"; do
    [[ -f "$f" ]] && { tr -d " \t\n\r" < "$f"; return; }
  done
  [[ -n "${HCLOUD_TOKEN:-}" ]] && { printf '%s' "$HCLOUD_TOKEN"; return; }
  echo "no token: put it in ~/hetzner.api (chmod 600) or export HCLOUD_TOKEN" >&2; exit 1
}
api() { # api METHOD PATH [json]
  local m=$1 p=$2 body=${3:-}
  if [[ -n "$body" ]]; then
    curl -sS -X "$m" -H "Authorization: Bearer $(token)" -H 'Content-Type: application/json' \
         -d "$body" "$API$p"
  else
    curl -sS -X "$m" -H "Authorization: Bearer $(token)" "$API$p"
  fi
}
server_json() { api GET "/servers?name=$NAME" | jq '.servers[0] // empty'; }
server_ip()   { server_json | jq -r '.public_net.ipv4.ip // empty'; }
# Hetzner RECYCLES IPs. A new box on an old IP presents a new host key, and
# StrictHostKeyChecking=accept-new accepts NEW keys but (rightly) REFUSES CHANGED ones -- so
# `waiting for ssh` spins until the timeout while the box sits there billing. Drop the stale
# key first. This is not a security relaxation: the key genuinely did change, because it is
# genuinely a different machine.
ssh_forget()  { ssh-keygen -R "$(server_ip)" >/dev/null 2>&1 || true; }
ssh_run()     { ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "root@$(server_ip)" "$@"; }

cmd_price() {
  # Ask the API rather than trusting anyone's memory of the price list.
  echo "CCX line (dedicated vCPU) — live from the Hetzner API:"
  api GET "/server_types?per_page=100" | jq -r '
    .server_types[] | select(.name|startswith("ccx")) |
    . as $t | ($t.prices[] | select(.location=="'"$LOCATION"'")) as $p |
    "  \($t.name)\t\($t.cores) vCPU\t\($t.memory|floor) GB\t€\(($p.price_hourly.gross|tonumber)|.*10000|round/10000)/hr\t€\(($p.price_monthly.gross|tonumber)|.*100|round/100)/mo"
  ' | column -t -s$'\t'
  echo
  echo "NOTE: billed while the server EXISTS. Powering it off changes nothing. Delete it."
}

cmd_up() {
  local type=${1:-$TYPE_DEFAULT}
  if [[ -n "$(server_json)" ]]; then echo "'$NAME' already exists at $(server_ip)"; return; fi

  # Find the key by its CONTENT, not by a name we invented. The key may already be in the
  # project under some other name (it was: "msi-z790"), in which case a name lookup misses
  # it, the re-upload fails `uniqueness_error`, and the server create then references a key
  # name that does not exist -> "SSH key not found".
  local pub keyid
  pub=$(awk '{print $1" "$2}' "$SSH_KEY_FILE")
  keyid=$(api GET "/ssh_keys?per_page=100" \
          | jq -r --arg p "$pub" '.ssh_keys[] | select((.public_key|split(" ")|.[0:2]|join(" "))==$p) | .id' \
          | head -1)
  if [[ -z "$keyid" ]]; then
    echo "uploading $SSH_KEY_FILE"
    keyid=$(api POST /ssh_keys "$(jq -n --arg n "exokey-$(hostname -s)-$$" --arg k "$(cat "$SSH_KEY_FILE")" \
      '{name:$n, public_key:$k}')" | jq -r '.ssh_key.id // empty')
    [[ -z "$keyid" ]] && { echo "could not register ssh key" >&2; exit 1; }
  fi
  echo "using ssh key id $keyid"

  echo "creating $type ($IMAGE, $LOCATION) as '$NAME'..."
  local resp
  resp=$(api POST /servers "$(jq -n --arg n "$NAME" --arg t "$type" --arg i "$IMAGE" \
    --arg l "$LOCATION" --argjson k "$keyid" \
    '{name:$n, server_type:$t, image:$i, location:$l, ssh_keys:[$k], start_after_create:true}')")
  if jq -e '.error' >/dev/null <<<"$resp"; then
    echo "ERROR: $(jq -r '.error.message' <<<"$resp")" >&2; exit 1   # do NOT press on
  fi
  echo "created id $(jq -r '.server.id' <<<"$resp")"

  ssh_forget                      # Hetzner recycles IPs; a stale host key hangs this loop
  echo -n "waiting for ssh"
  for _ in $(seq 60); do
    if ssh_run true 2>/dev/null; then echo " — up at $(server_ip)"; break; fi
    echo -n .; sleep 5
  done

  echo "installing deps..."
  ssh_run 'export DEBIAN_FRONTEND=noninteractive; apt-get -qq update &&
           apt-get -qq install -y python3-venv python3-dev build-essential rsync >/dev/null'
  echo "pushing repo..."
  rsync -az --delete \
    --exclude .venv --exclude out --exclude '__pycache__' --exclude '.pytest_cache' \
    "$REPO_DIR/" "root@$(server_ip):/opt/exokey/"
  ssh_run 'cd /opt/exokey && python3 -m venv .venv &&
           .venv/bin/pip -q install --upgrade pip &&
           .venv/bin/pip -q install mujoco numpy scipy pymoo PyNiteFEA plotly pytest'
  echo "verifying the physics survived the trip (52 tests)..."
  ssh_run 'cd /opt/exokey && PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python -m pytest tests/ -q 2>&1 | tail -1'
  echo "ready.  ./cloud/hetzner.sh run --pop 200 --gen 150"
}

cmd_run() {
  local n; n=$(ssh_run nproc)
  local procs=$(( n > 2 ? n - 1 : 1 ))   # dedicated box, nothing else on it
  # Multi-start: independent seeds explore different basins, and if two seeds disagree about
  # what is on the front then NEITHER has converged. One box, both seeds, one teardown.
  local seeds=${EXOKEY_SEEDS:-1}
  echo "running on $n vCPU (--procs $procs), seeds: $seeds"
  # OMP_NUM_THREADS=1 is not optional: N workers x N BLAS threads oversubscribes the box
  # and the "parallel" run comes out slower than serial.
  for sd in $seeds; do
    echo "--- seed $sd ---"
    ssh_run "cd /opt/exokey && mkdir -p out && PYTHONPATH=. OMP_NUM_THREADS=1 \
             EXOKEY_OUT=out/pareto_seed${sd}.pkl \
             .venv/bin/python -u -m opt.run --procs $procs --seed $sd $* 2>&1 \
             | tee out/nsga_seed${sd}.log"
  done
}

cmd_fetch() { rsync -az "root@$(server_ip):/opt/exokey/out/" "$REPO_DIR/out/"; echo "pulled -> out/"; }

cmd_burn() {
  # Run, pull the results back, then DELETE the box. The whole point of a burst machine is
  # that it stops existing. Forgetting to delete it is the only way this gets expensive --
  # a powered-off Hetzner Cloud server bills exactly the same as a running one -- so the
  # teardown is wired to the run instead of left to memory.
  #
  # The trap fires on error and on interrupt too: a crashed run must not leave a box billing.
  trap 'echo "--- tearing down (trap) ---"; cmd_fetch 2>/dev/null || true; cmd_force_down' EXIT INT TERM
  cmd_run "$@"
  trap - EXIT INT TERM
  echo "--- run finished; fetching and tearing down ---"
  cmd_fetch
  cmd_force_down
}

cmd_force_down() {  # no prompt: the caller already committed to burning the box
  local id; id=$(server_json | jq -r '.id // empty')
  [[ -z "$id" ]] && { echo "nothing to delete"; return; }
  api DELETE "/servers/$id" >/dev/null
  echo "deleted '$NAME'. billing stopped."
}

cmd_status() {
  local s; s=$(server_json)
  [[ -z "$s" ]] && { echo "no server named '$NAME' — you are being billed nothing"; return; }
  jq -r '"\(.name)  \(.server_type.name)  \(.server_type.cores) vCPU  \(.status)  \(.public_net.ipv4.ip)  created \(.created)"' <<<"$s"
  echo "⚠ still billing (status is irrelevant — only deleting stops it).  ./cloud/hetzner.sh down"
}

cmd_down() {
  local id; id=$(server_json | jq -r '.id // empty')
  [[ -z "$id" ]] && { echo "nothing to delete"; return; }
  read -rp "DELETE server '$NAME' (id $id)? results not fetched are lost. [y/N] " a
  [[ "$a" == "y" ]] || { echo "aborted"; return; }
  api DELETE "/servers/$id" >/dev/null
  echo "deleted. billing stopped."
}

cmd_run_bg() {
  # DETACHED run. `burn` tethers the optimiser to an SSH session on THIS machine, so a local
  # reboot SIGHUPs the remote run AND may leave the box billing (the teardown is driven from
  # here). Here the run is `setsid`-detached on the box with its stdio redirected, so it outlives
  # the ssh channel, this shell, and a reboot of this laptop. A DONE marker signals completion;
  # `watch` fetches + deletes only then. Teardown is deliberately NOT wired to this command --
  # the box must survive this machine.
  local n; n=$(ssh_run nproc)
  local procs=$(( n > 2 ? n - 1 : 1 ))
  local sd=${EXOKEY_SEEDS:-1}
  echo "detaching run on $(server_ip): $n vCPU (--procs $procs), seed $sd, args: $*"
  ssh_run "cd /opt/exokey && mkdir -p out && rm -f out/DONE && \
    setsid sh -c 'PYTHONPATH=. OMP_NUM_THREADS=1 EXOKEY_OUT=out/pareto_seed${sd}.pkl \
      .venv/bin/python -u -m opt.run --procs $procs --seed $sd $* > out/nsga_seed${sd}.log 2>&1; \
      touch out/DONE' >/dev/null 2>&1 < /dev/null &"
  sleep 2
  echo "started -- it now runs independently of this machine."
  echo "  progress:  ./cloud/hetzner.sh tail"
  echo "  finish:    ./cloud/hetzner.sh watch    (fetches out/ and DELETES the box when done)"
}

cmd_tail() {
  local sd=${EXOKEY_SEEDS:-1}
  ssh_run "test -f /opt/exokey/out/DONE && echo '[DONE]'; \
           tail -n 25 /opt/exokey/out/nsga_seed${sd}.log 2>/dev/null || echo 'no log yet'"
}

cmd_watch() {
  # Poll the DETACHED run, then fetch + DELETE. Safe to Ctrl-C and safe to lose (reboot): it does
  # NOT tear down on interrupt, only on genuine completion. So a reboot of this machine leaves the
  # remote run and box untouched -- just run `watch` again when you are back.
  [[ -z "$(server_json)" ]] && { echo "no box named '$NAME' -- nothing to watch"; return; }
  echo "polling $(server_ip) for out/DONE (Ctrl-C is safe; the box keeps running)..."
  while true; do
    [[ -z "$(server_json)" ]] && { echo "the box is gone -- stopping"; return; }
    ssh_run "test -f /opt/exokey/out/DONE" 2>/dev/null && break
    sleep 60
  done
  echo "run finished. fetching + tearing down."
  cmd_fetch
  cmd_force_down
}

case "${1:-}" in
  price) shift; cmd_price "$@" ;;
  burn) shift; cmd_burn "$@" ;;
  up) shift; cmd_up "$@" ;;
  run) shift; cmd_run "$@" ;;
  run-bg) shift; cmd_run_bg "$@" ;;
  watch) shift; cmd_watch "$@" ;;
  tail) shift; cmd_tail "$@" ;;
  fetch) shift; cmd_fetch "$@" ;;
  status) shift; cmd_status "$@" ;;
  down) shift; cmd_down "$@" ;;
  *) sed -n '2,16p' "$0" ;;
esac
