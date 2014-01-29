set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibrated-Place-synapse-final.eps"
set xlabel "Place cells"
set ylabel "Synaptic weight"
set title ""
splot "./Calibrated-Place-synapse-final.txt" matrix with lines title "Recurrent synaptic weights for place cells"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibrated-Place-synapse-final-normalized.eps"
set xlabel "Place cells"
set ylabel "Synaptic weight"
set title ""
splot "./Calibrated-Place-synapse-final-normalized.txt" matrix with lines title "Normalized recurrent synaptic weights for place cells"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibrated-Place-synapse-N.eps"
set xlabel "Place cells"
set ylabel "Synaptic weight"
set title ""
splot "./Calibrated-Place-synapse-N.txt" matrix with lines title "recurrent synaptic weights for place cells N"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibrated-Place-synapse-S.eps"
set xlabel "Place cells"
set ylabel "Synaptic weight"
set title ""
splot "./Calibrated-Place-synapse-S.txt" matrix with lines title "recurrent synaptic weights for place cells S"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibrated-Place-synapse-E.eps"
set xlabel "Place cells"
set ylabel "Synaptic weight"
set title ""
splot "./Calibrated-Place-synapse-E.txt" matrix with lines title "recurrent synaptic weights for place cells E"
set term wxt
set out

set title
set xlabel
set ylabel
set zlabel

