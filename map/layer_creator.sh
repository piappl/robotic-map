#!/bin/bash -i
set -e

if [ "$#" -ne 5 ]; then
  echo "Provide input file, layer name, blank tile path, min and max zoom (i.e. ./map_deploy.sh map.tif aerial_20140702 icons/blankTile.png 12 18)"
  exit 1
fi

userhome=$(eval echo ~${SUDO_USER})
deployDir="$userhome/.local/share/marble/maps/earth/$2"

gdal2tiles.py -z $4-$5 -n $1 $deployDir
mkdir $deployDir/0
mkdir $deployDir/0/0
cp $3 $deployDir/0/0
rm $deployDir/googlemaps.html $deployDir/openlayers.html $deployDir/tilemapresource.xml

echo "Finished"
