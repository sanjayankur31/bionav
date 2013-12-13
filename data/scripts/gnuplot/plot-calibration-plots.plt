set terminal postscript eps enhanced color font "Helvetica,8" linewidth 1 
set output "Calibration-synaptic-weight-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-HD-synapse-final-50-row.txt" with lines title "Synaptic weights for head direction cell 50 combined"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,8" linewidth 1 
set output "Calibration1-synaptic-weight-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated1-HD-synapse-50-row.txt" with lines title "Synaptic weights for head direction cell 50 after counter clockwise training"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,8" linewidth 1 
set output "Calibration2-synaptic-weight-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated2-HD-synapse-50-row.txt" with lines title "Synaptic weights for head direction cell 50 after clockwise training"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,8" linewidth 1 
set output "Calibration-synaptic-weight-rotation-cell-clockwise-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated-HD-RotationCellClockwise-synapse-final-cell50.txt" with lines title "Effective synaptic weights for clockwise rotation cell and head direction cell 50"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,8" linewidth 1 
set output "Calibration-synaptic-weight-rotation-cell-counterclockwise-cell50.eps"
set xlabel "Head direction cells"
set ylabel "Synaptic weight"
set title ""
plot "./Calibrated1-HD-RotationCellCounterClockwise-synapse-cell50.txt" with lines title "Effective synaptic weights for counter clockwise rotation cell and head direction cell 50"
set term wxt
set out

set terminal postscript eps enhanced color font "Helvetica,8" linewidth 1 
set output "Calibration-synaptic-weight-surface-plot.eps"
set xlabel "Head direction cells"
set ylabel "Head direction cells"
set zlabel "Synaptic \nweight"
set view 52,115
splot "./Calibrated-HD-synapse-final.txt" matrix with lines title "Synaptic weight"
set title ""
set term wxt
set out
set title
set xlabel
set ylabel
set zlabel

