#!/bin/sh
sed -i 's/TEO_ERROR/CD_ERROR/g' *.hpp *.cpp
sed -i 's/TEO_WARNING/CD_WARNING/g' *.hpp *.cpp
sed -i 's/TEO_SUCCESS/CD_SUCCESS/g' *.hpp *.cpp
sed -i 's/TEO_INFO/CD_INFO/g' *.hpp *.cpp
sed -i 's/TEO_DEBUG/CD_DEBUG/g' *.hpp *.cpp

