#!/bin/bash

for f in *.csv; 
do 
    sed -e "s/INPUT/$f/" -e "s/OUTPUT/${f/csv/png}/" uwbplot.gpl | gnuplot -; 
done

