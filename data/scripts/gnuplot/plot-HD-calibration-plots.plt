set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibration-HD-synaptic-weight-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-HD-synapse-final-50-row.txt" with lines title "Synaptic weights for head direction cell 50"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibration1-HD-synaptic-weight-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-HD-synapse-1-50.txt" with lines title "Synaptic weights after counter clockwise training"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibration2-HD-synaptic-weight-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-HD-synapse-2-50.txt" with lines title "Synaptic weights after clockwise training"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibration-HD-synaptic-weight-rotation-cell-clockwise-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-RotationCellClockwise-synapse-cell50.txt" with lines title ""
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,20" linewidth 1 
set output "Calibration-HD-synaptic-weight-rotation-cell-counterclockwise-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-RotationCellCounterClockwise-synapse-cell50.txt" with lines title ""
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,16" linewidth 1 
set output "Calibration-HD-synaptic-weight-surface-plot.eps"
set xlabel "Head direction cells"
set ylabel "Head direction cells"
set zlabel "Synaptic \nweight"
set view 52,115
#splot "./Calibrated-HD-synapse-final.txt" matrix with lines title "Synaptic weight"
splot "./Calibrated-HD-synapse-final.txt" matrix with lines title ""
set title ""
set term wxt
set out
set title
set xlabel
set ylabel
set zlabel

