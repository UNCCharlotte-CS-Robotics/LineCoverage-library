#!/bin/bash

print_usage() {
	printf "bash $0 [-alkh] [-aglkh]\n"
}

USE_LKH=false
USE_GLKH=false
while getopts 'd:a:' flag; do
	case "${flag}" in
		d) LC_DATABASE_DIR="${OPTARG%/}" ;;
		a) if [[ ${OPTARG} == "lkh" ]]
			then
				USE_LKH=true
			fi
			if [[ ${OPTARG} == "glkh" ]]
			then
				USE_GLKH=true
				USE_LKH=true
			fi
			;;
		*) print_usage
			exit 1 ;;
esac
done
BASE_DIR=`pwd`
source ~/.bashrc

LC_EXT_DIR=${BASE_DIR}/
LOG_FILE=${LC_EXT_DIR}/lkh_glkh_setup.log

cd ${LC_EXT_DIR}

if [[ ${USE_LKH} == true ]]
then
	echo "===================="
	echo ""
	echo "Obtaining LKH"
	echo ""
	echo "===================="
	echo ""
	wget -O lkh.tgz http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.8.tgz
	tar -xzf lkh.tgz -C ${LC_EXT_DIR}
	rm lkh.tgz
	cd ${LC_EXT_DIR}
	mv LKH-3.0.8 lkh
fi

if [[ ${USE_GLKH} == true ]]
then
	echo "===================="
	echo ""
	echo "Obtaining GLKH"
	echo ""
	echo "===================="
	echo ""

	wget http://akira.ruc.dk/~keld/research/GLKH/GLKH-1.1.tgz
	mkdir -p ${LC_EXT_DIR}
	tar -xzf GLKH-1.1.tgz -C ${LC_EXT_DIR}
	rm GLKH-1.1.tgz
	mv GLKH-1.1 glkh
	sed -i ' 4 s/.*/& -fcommon/' glkh/SRC/Makefile
	rm glkh/SRC/GLKH_EXPmain.c
	rm glkh/SRC/GLKH_CHECKmain.c

	cd ${LC_EXT_DIR}/glkh/
	GLKH_SCRIPT=glkh.sh

	# SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
	echo '#!/bin/bash' > $GLKH_SCRIPT
	# echo "cd "$(dirname ${BASH_SOURCE[0]})"" >> $GLKH_SCRIPT
	echo 'cd ${1}' >> $GLKH_SCRIPT
	echo './GLKH TMP/gtsp.par' >> $GLKH_SCRIPT
	# echo "rm -r TMP/*" >> $GLKH_SCRIPT
fi
