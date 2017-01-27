#!/bin/bash

REGS="bcm_regs.csv"
TEMPL="SIS8300bcm.template"
OPI="SIS8300bcmRegs.opi"
WIDGETS="widgets.opi"

# label
LX=5
LW=295
# input
IX=300
IW=95
# update
UX=400
UW=95
# button
BX=500
BW=54
# common
Y=5
H=20

function templ() {
	#echo -e "$*"
	echo -e "$*" >> $TEMPL
}

function opi_label() {
	sed -e "s/%N%/$1/" \
		-e "s/%X%/$LX/" \
		-e "s/%Y%/$Y/" \
		-e "s/%H%/$H/" \
		-e "s/%W%/$LW/" \
	label.stub >> $WIDGETS
}

function opi_textinput() {
	sed -e "s/%PV%/$1/" \
		-e "s/%X%/$IX/" \
		-e "s/%Y%/$Y/" \
		-e "s/%H%/$H/" \
		-e "s/%W%/$IW/" \
	textinput.stub >> $WIDGETS
}

function opi_textupdate() {
	sed -e "s/%PV%/$1/" \
		-e "s/%X%/$UX/" \
		-e "s/%Y%/$Y/" \
		-e "s/%H%/$H/" \
		-e "s/%W%/$UW/" \
	textupdate.stub >> $WIDGETS
}

function opi_button() {
	sed -e "s/%PV%/$1/" \
		-e "s/%X%/$BX/" \
		-e "s/%Y%/$Y/" \
		-e "s/%H%/$H/" \
		-e "s/%W%/$BW/" \
	button.stub >> $WIDGETS
}

function pv_out() {
	templ "record(longout, \"\$(P)\$(R)$1\")"
	templ "{"
	templ "    field(PINI, \"YES\")"
	templ "    field(DTYP, \"asynInt32\")"
	templ "    field(OUT,  \"@asyn(\$(PORT),\$(ADDR),\$(TIMEOUT))$2 $3\")"
	templ "    info(autosaveFields, \"VAL\")"
	templ "}"
}

function pv_in () {
	templ "record(longin, \"\$(P)\$(R)${1}_RBV\")"
	templ "{"
	templ "    field(DTYP, \"asynInt32\")"
	templ "    field(INP,  \"@asyn(\$(PORT),\$(ADDR),\$(TIMEOUT))$2 $3\")"
	templ "    field(SCAN, \"I/O Intr\")"
	templ "}"
}


rm -f $TEMPL $OPI $WIDGETS

templ "# Autogenerated list of PVs for BCM firmware SAT"
templ "# Author: Hinko Kocevar <hinko.kocevar@esss.se>"
templ "# Date:   $(date)"
templ ""

for l in $(cat $REGS); do
	R=$(echo "$l" | cut -f1 -d,)
	A=$(echo "$l" | cut -f2 -d,)
	D=$(echo "$l" | cut -f3 -d,)
	
	templ "## Register '$R' is $A ($D)"
	
	echo "$R" | grep -q 0xn
	if [ $? -eq 0 ]; then
		C=0
		for i in 5 6 7 8 9 A B C D E; do
			RR=$(echo "$R" | sed -e "s/0xn/0x$i/")
			N=$(echo "$D" | sed -e "s/BCM_//")
			NN="${N}_CH$C"
			DD="${D}_CH$C"
			opi_label "$RR $N"
			if [ "$A" == "WO" -o "$A" == "RW" ]; then
				pv_out $NN $DD $RR
				opi_textinput "$NN"
			fi
			if [ "$A" == "RO" -o "$A" == "RW" ]; then
				pv_in $NN $DD $RR
				opi_textupdate "$NN"
				opi_button "$NN"
			fi
			templ ""
			C=$((C+1))
			Y=$((Y + H + 5))
		done
	else
		RR="$R"
		N=$(echo "$D" | sed -e "s/BCM_//")
		NN="${N}"
		DD="${D}"
		opi_label "$RR $N"
		if [ "$A" == "WO" -o "$A" == "RW" ]; then
			pv_out $NN $DD $RR
			opi_textinput "$NN"
		fi
		if [ "$A" == "RO" -o "$A" == "RW" ]; then
			pv_in $NN $DD $RR
			opi_textupdate "$NN"
			opi_button "$NN"
		fi
		templ ""
		Y=$((Y + H + 5))
	fi
done

cp panel_pre.stub $OPI
HH=$((Y + H + 5))
WW=$((BX + BW + 6))
sed -i -e "s/%H%/$HH/" \
	-e "s/%W%/$WW/" \
	$OPI
	
cat $WIDGETS >> $OPI
cat panel_post.stub >> $OPI

C=$(grep '^record' $TEMPL | wc -l)
echo "Created $C records in $TEMPL"
C=$(grep '^  <widget' $OPI | wc -l)
echo "Created $C widgets in $OPI"
