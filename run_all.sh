#!/usr/bin/env bash
# Όλες οι δοκιμές. Χρήση:  bash tests/run_all.sh  [φάκελος με πραγματικά DXF]
set -e
cd "$(dirname "$0")/.."
echo "── build ─────────────────────────────────────────────"
python3 build/build_index.py
node --check build/_extracted.js && echo "  σύνταξη JS: ΟΚ"
echo "── συνθετικά DXF (R12) ───────────────────────────────"
rm -rf tests/fixtures
python3 tests/gen_test_dxf.py
node tests/test_merge.js
node tests/test_merge2.js
echo "── end-to-end της σελίδας (χωρίς browser) ────────────"
node tests/test_app.js | tail -6
if [ -n "$1" ]; then
  echo "── πραγματικό έργο: $1 ───────────────────────────────"
  node tests/check_clearance.js "$1"
  node tests/run_project.js "$1" /tmp/ΕΝΟΠΟΙΗΜΕΝΟ_test.dxf
fi
echo "ΟΛΑ ΟΚ"
