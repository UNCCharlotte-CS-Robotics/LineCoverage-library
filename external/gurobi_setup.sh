#!/bin/bash

# Gurobi installation script

INSTALL_DIR=${HOME}/opt/
MAJOR_VERSION=10.0
MINOR_VERSION=0
mkdir -p ${INSTALL_DIR}
cd ${INSTALL_DIR}
GUROBI_FILENAME=gurobi${MAJOR_VERSION}.${MINOR_VERSION}_linux64.tar.gz
wget https://packages.gurobi.com/${MAJOR_VERSION}/${GUROBI_FILENAME}
tar -xf ${GUROBI_FILENAME}
rm ${GUROBI_FILENAME}
GUROBI_LIB=gurobi"${MAJOR_VERSION//.}"
GUROBI_DIR=gurobi"${MAJOR_VERSION//.}"${MINOR_VERSION}
echo ${GUROBI_DIR}
cd ${GUROBI_DIR}/linux64/src/build
sed -i ' 1 s/.*/& -std=c++17/' Makefile
make -j
cp libgurobi_c++.a ../../lib/.

echo "export GUROBI_HOME=${INSTALL_DIR}/${GUROBI_DIR}/linux64" >> ${HOME}/.bashrc
echo 'export PATH="${PATH}:${GUROBI_HOME}/bin"' >> ${HOME}/.bashrc
echo 'export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"' >> ${HOME}/.bashrc
echo "export GUROBI_LIB=${GUROBI_LIB}" >> ${HOME}/.bashrc

source ${HOME}/.bashrc

echo "Gurobi installation is complete. Please activate your license. Visit www.gurobi.com"
