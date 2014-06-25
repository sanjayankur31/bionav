set terminal postscript eps enhanced color font "Helvetica,16" linewidth 1 
set output "Calibration-Grid-synaptic-weight-surface-plot.eps"
set xlabel "Grid cells"
set ylabel "Grid cells"
set zlabel "W"
set view 44,312
splot "./Calibrated-Grid-synapse-final-normalized.txt" matrix with lines title ""
set terminal postscript eps enhanced color font "Helvetica,16" linewidth 1 
set output "Calibration-Grid-HD25-synaptic-weight-surface-plot.eps"
set xlabel "Grid cells"
set ylabel "Grid cells"
set zlabel "W"
set view 44,312
splot "./Calibrated-Grid-HD25-V-synapse-normalized.txt" matrix with lines title ""
set title ""
set term wxt
set out
set title
set xlabel
set ylabel
set zlabel

