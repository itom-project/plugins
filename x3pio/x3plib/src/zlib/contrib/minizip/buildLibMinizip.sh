#! /bin/bash

# Check number of parameters
if [ -z $2 ]
then
  echo "Usage: buildLibMinizip.sh {Source Dir} {Dest Dir}
  This command will build the libminizp and copy the result to \" Dest Dir \" "
  exit 1
fi

# Base dir of project
srcDir="$1"
destDir="$2"
oldDir=$(pwd)

cd $srcDir

set CPPFLAGS="-DUNICODE -D_UNICODE"

echo "running libtoolize"
libtoolize

echo "running aclocal"
aclocal

echo "running autoconf"
autoconf

echo "running automake --add-missing"
automake --add-missing

echo "running automake"
automake

echo "running configure with --enable-shared"
./configure --enable-shared

#make clean
echo "building libminizip"
make

echo "copying libminizip to " $destDir
cp $srcDir/.libs/*.so* $destDir/

cd $oldDir
