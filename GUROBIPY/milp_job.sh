#!/usr/bin/env bash
#SBATCH -J sh
#SBATCH -n 12
#SBATCH -N 1
#SBTACH -p short
#SBATCH --mem 32G

# Stop execution after any error
set -e

# Cleanup function to be executed upon exit, for any reason
function cleanup() {
    rm -rf $WORKDIR
}



########################################
#
# Useful variables
#
########################################

# Your user name
# (Don't change this)
MYUSER=$(whoami)

# Path of the local storage on a node
# Use this to avoid sending data streams over the network
# (Don't change this)
LOCALDIR=/local
# To be changed as per experiment (my cluster environment)
MYDIR=~/work/topf_3_20

# Folder where you want your data to be stored (my cluster environment)
DATADIR=$MYDIR/data

# Change as per experiment
THISJOB=~/work

########################################
#
# Job-related variables
#
########################################

# Job working directory
# (Don't change this)
WORKDIR=$LOCALDIR/$MYUSER/$THISJOB



########################################
#
# Job directory
#
########################################

# Create work dir from scratch, enter it
# (Don't change this)
rm -rf $WORKDIR && mkdir -p $WORKDIR && cd $WORKDIR

# Make sure you cleanup upon exit
# (Don't change this)
trap cleanup EXIT SIGINT SIGTERM



########################################
#
# Actual job logic
#
########################################

# Execute job
# Commands
module load gurobi
python3 $MYDIR/main.py

# Transfer generated *.dat files into home directory
# Create the two folders (my cluster environment)
cp -a *.csv $DATADIR/csv
cp -a *.html $DATADIR/img

