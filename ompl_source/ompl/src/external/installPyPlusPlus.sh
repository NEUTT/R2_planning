#!/bin/sh

set -e

# create a location to store downloaded data
mkdir -p /home/tt-lab/Documents/ros_workspace/build_isolated/ompl_source/devel/pyplusplus
cd /home/tt-lab/Documents/ros_workspace/build_isolated/ompl_source/devel/pyplusplus

# get sources
# gccxml snapshot of 5/2/2013
/usr/bin/curl --location-trusted https://github.com/gccxml/gccxml/archive/b040a46352e4d5c11a0304e4fcb6f7842008942a.tar.gz | tar xzf -
/usr/bin/curl --location-trusted https://bitbucket.org/ompl/pygccxml/downloads/pygccxml-r579.tgz | tar xzf -
/usr/bin/curl --location-trusted https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1246.tgz | tar xzf -

# build & install gccxml
cd gccxml-b040a46352e4d5c11a0304e4fcb6f7842008942a
/usr/bin/cmake  .
/usr/bin/cmake --build .
 /usr/bin/cmake --build . --target install

# build & install pygccxml and Py++
cd ../pygccxml
 setup.py build
  setup.py install 
cd ../pyplusplus
 setup.py build
  setup.py install 
