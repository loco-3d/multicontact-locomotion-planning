#!/bin/bash

# Exit on error
set -e

cd $DEVEL_HPP_DIR/src
make hpp-gepetto-viewer.install
echo "export CORBA_HOST=172.17.0.1" >> ~/.bashrc
cd $DEVEL_HPP_DIR/src
git clone --recursive https://git-amd.tuebingen.mpg.de/bponton/timeoptimization
cd timeoptimization/
./src/catkin/third_party/catkin/bin/catkin_make
source ./devel/setup.bash
echo "source $DEVEL_HPP_DIR/src/timeoptimization/devel/setup.bash " >> ~/.bashrc
cp $DEVEL_HPP_DIR/src/timeoptimization/devel/lib/pkgconfig/yaml_cpp.pc $DEVEL_HPP_DIR/src/timeoptimization/devel/lib/pkgconfig/yaml-cpp.pc
cd $DEVEL_HPP_DIR/src
git clone https://github.com/ggory15/timeopt --recursive
mkdir timeopt/build
cd timeopt/build
cmake -DCMAKE_INSTALL_PREFIX=$DEVEL_HPP_DIR/install/ -DCMAKE_BUILD_TYPE=Release ..
make install
cd $DEVEL_HPP_DIR/src
git clone --recursive https://github.com/stack-of-tasks/tsid.git
mkdir tsid/build
cd tsid/build
cmake -DCMAKE_INSTALL_PREFIX=$DEVEL_HPP_DIR/install/ -DCMAKE_BUILD_TYPE=Release ..
make install
cd $DEVEL_HPP_DIR/src
sudo apt-get install clang
cd pinocchio/build-rel
cmake -DBUILD_PYTHON_INTERFACE=ON ..
make install
cd $DEVEL_HPP_DIR/src
git clone --recursive https://gepgitlab.laas.fr/loco-3d/multicontact-api.git
mkdir multicontact-api/build
cd multicontact-api/build
cmake -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang -DCMAKE_INSTALL_PREFIX=$DEVEL_HPP_DIR/install/ -DCMAKE_BUILD_TYPE=Release ..
make install
cd $DEVEL_HPP_DIR/src
git clone https://gepgitlab.laas.fr/pfernbac/hpp-wholebody-motion.git
mkdir -p hpp-wholebody-motion/res/contact_sequences
cd $DEVEL_HPP_DIR/src
mkdir logs
export PYTHONPATH=$PYTHONPATH:$DEVEL_HPP_DIR/src/hpp-rbprm-corba/script
export PYTHONPATH=$PYTHONPATH:$DEVEL_HPP_DIR/src/hpp-wholeboy-motion/scripts
echo "export PYTHONPATH=$PYTHONPATH:$DEVEL_HPP_DIR/src/hpp-rbprm-corba/script" >>  ~/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:$DEVEL_HPP_DIR/src/hpp-wholeboy-motion/scripts" >>  ~/.bashrc








