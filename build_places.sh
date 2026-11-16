#!/bin/sh
# Builds the four test places and runs every check on each (real heights, repeat, shifted area, 3x hills).
# Usage: ./build_places.sh            (needs data/raw with the map extracts and elevation tiles, see README)
set -u
V=engine/target/release/voxbox
status=0
run() {
  $V build "$1" "$2" "$3" 256 "data/raw/$4.osm.pbf" data/raw "places/$1" --check ${5:-} || status=1
}
run lisbon_alfama      38.7118  -9.1300 Lisbon --x3
run copenhagen_centre  55.6795  12.5760 Copenhagen
run innsbruck_hoetting 47.2735  11.3830 Innsbruck
run tyrol_country_road 47.2500  11.4650 Innsbruck
exit $status
