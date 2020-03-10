#!/bin/bash

# first two args are
INFILE=$1
OUTFILE=$2
shift
shift

# construct script in /tmp
SCRIPTFILE=/tmp/meshlab_run.mlx

# add header
echo "<!DOCTYPE FilterScript>" > $SCRIPTFILE
echo "<FilterScript>" >> $SCRIPTFILE

# always start by removing duplicates - quadric decimation doesn't work
# without this, maybe other filters too
echo "<filter name=\"Remove Duplicated Vertex\"/>" >> $SCRIPTFILE

# for each argument after the first two (INFILE, OUTFILE)
while [[ "$#" != "0" ]]; do

	# split SCRIPT=ARGS
	SCRIPT=meshlab_script/${1%=*}.mlx
	ARGS=${1#*=}
	shift
	
	# split SCRIPT=ARG1,ARG2,ARG3
	ARGARR=(${ARGS//,/ })
	
	# check script exists
	if [[ ! -e "$SCRIPT" ]];
	then
		echo "script not found: $SCRIPT"
		exit 1
	fi
	
	# add script component to /tmp script, replacing args
	cat $SCRIPT | sed \
		-e s/\$ARGS/$ARGS/g \
		-e s/\$ARG1/${ARGARR[0]}/g \
		-e s/\$ARG2/${ARGARR[1]}/g \
		-e s/\$ARG3/${ARGARR[2]}/g \
		>> $SCRIPTFILE
	
done

# add footer
echo "</FilterScript>" >> $SCRIPTFILE

# optional args
if [[ "${OUTFILE: -4}" == ".dae" ]];
then
OUTPUTS="-om vn"
else
OUTPUTS=
fi

# run script
meshlabserver -i "$INFILE" -o "$OUTFILE" $OUTPUTS -s "$SCRIPTFILE" &> /dev/null




