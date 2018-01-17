#!/bin/bash -i
set -e

if [ "$#" -ne 1 ]; then
  echo "Provide Cmake Prefix Path for QT (i.e. ~/Qt/5.9.3/gcc_64/lib/cmake/)"
  exit 1
fi

scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
userhome=$(eval echo ~${SUDO_USER})
mapwidgetMarblePatchPath="${scriptdir}/marblePatch.diff"
installPrefix="/usr/local"
cmakeModulePath=${installPrefix}/share/marble/cmake
cmakePrefixPath=$1

echo "****Fetching marble from repository"

marbleDir="$userhome/marble"
marbleSource="$marbleDir/source"
marbleBuildDir="$marbleDir/build"

mkdir -p $marbleDir

if [ ! -d "$marbleSource" ]; then
   cd $marbleDir
   git clone git://anongit.kde.org/marble $marbleSource
   cd $marbleSource
   echo "****git update and checkout to supported version"
   git remote update
   git checkout 0bc8ff
   echo "****Applying patch"
   git apply $mapwidgetMarblePatchPath
   git add --all
   git commit -m "mapwidget patch"
   git checkout -b mapwidget_branch
else
   echo "****Found marble dir at $marbleSource, assuming git repo there and up-to-date sources (if not sure, delete contents and start over)"
fi

if [ ! -d "$marbleBuildDir" ]; then
   mkdir -p $marbleBuildDir
else
   echo "****Found marble build dir at $marbleBuildDir"
fi

echo "****Building marble"
cd $marbleBuildDir
cmake -DCMAKE_BUILD_TYPE=Debug -DQTONLY=1 -DCMAKE_INSTALL_PREFIX=$installPrefix -DCMAKE_PREFIX_PATH=$cmakePrefixPath -DBUILD_MARBLE_APPS=OFF -DBUILD_WITH_DBUS=0 -DQT_NO_DBUS=1 $marbleSource
make -j 8

echo "****Installing marble"
sudo make install

echo "****Marble installation complete!"
echo ""
echo "                    |_|_|_|_|"
echo "                   /         \\"
echo "                  |  0     0  |"
echo " MARBLE OK        |     V     |          MARBLE OK"
echo "                  |   \___/   |"
echo "                   \_________/"
echo "                      |   |   "
echo ""

mapBuildDir="$scriptdir/../build"

if [ ! -d "$mapBuildDir" ]; then
   mkdir -p $mapBuildDir
else
   echo "****Found map build dir at $mapBuildDir"
fi

echo "****Building map"
cd $mapBuildDir
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=$installPrefix -DCMAKE_PREFIX_PATH=$cmakePrefixPath -DCMAKE_MODULE_PATH=$cmakeModulePath $scriptdir
make -j 8

echo "****Installing map"
sudo make install

echo "****Copying default local map if none is there yet"
sudo mkdir -p "/share/maps/customicons"
sudo mkdir -p "/share/maps/localmap"
sudo cp -nr "${scriptdir}/localmap/." "/share/maps/localmap/"

echo "****Finished!"

