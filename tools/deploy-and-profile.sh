#!/bin/bash
# deploy-and-profile.sh — deploy the robot code, then live-tail the roboRIO
# program log over SSH to watch loop overruns as they happen.
#
# Usage:
#   ./tools/deploy-and-profile.sh            # default team 6995
#   TEAM=254 ./tools/deploy-and-profile.sh   # other team number
#
# On the roboRIO, watch these topics live instead with:
#   ssh admin@10.69.95.2 and open /LoopTiming in AdvantageScope.

set -euo pipefail
cd "$(dirname "$0")/.."

TEAM="${TEAM:-6995}"
RIO_HOST="roboRIO-${TEAM}-FRC.local"
RIO_IP="10.$((TEAM / 100)).$((TEAM % 100)).2"
RIO_LOG="/home/lvuser/logs/FRC_UserProgram.log"

echo "==> Deploying to roboRIO (${TEAM}) ..."
./gradlew deploy

echo "==> Program log at ${RIO_LOG} (tailing; Ctrl-C to stop)"
echo "    Tip: open NetworkTables '/LoopTiming' in AdvantageScope for graphs."
# Prefer the IP; fall back to the mDNS hostname if DNS is flaky.
ssh -o ConnectTimeout=5 "admin@${RIO_IP}" \
    "tail -f ${RIO_LOG} 2>/dev/null | grep --line-buffered -iE 'overrun|error|exception' || \
     journalctl -f -u frc-user-program" \
  || ssh -o ConnectTimeout=5 "admin@${RIO_HOST}" \
    "tail -f ${RIO_LOG} 2>/dev/null | grep --line-buffered -iE 'overrun|error|exception' || \
     journalctl -f -u frc-user-program"
