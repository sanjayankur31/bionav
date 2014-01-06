set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "01-Stabilization-firing-rates.eps"
set xlabel "Head direction cells"
set ylabel "Firing rate"
set title ""
plot "./01-Stabilized-HDCells-FiringRate-0.txt" with lines title "At time 0", "./01-Stabilized-HDCells-FiringRate-9.txt" with lines title "At time 10", "./01-Stabilized-HDCells-FiringRate-99.txt" with lines title "At time 100", "./01-Stabilized-HDCells-FiringRate-199.txt" with lines title "At time 200"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "01-Stabilization-activation.eps"
set xlabel "Head direction cells"
set ylabel "Activation"
set title ""
plot "./01-Stabilized-HDCells-Activation-0.txt" with lines title "At time 0", "./01-Stabilized-HDCells-Activation-9.txt" with lines title "At time 10", "./01-Stabilized-HDCells-Activation-99.txt" with lines title "At time 100", "./01-Stabilized-HDCells-Activation-199.txt" with lines title "At time 200"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "250.000000-Stabilization-activation.eps"
set xlabel "Head direction cells"
set ylabel "Activation"
set title ""
plot "./250.000000-Stabilized-HDCells-Activation-0.txt" with lines title "At time 0", "./250.000000-Stabilized-HDCells-Activation-9.txt" with lines title "At time 10", "./250.000000-Stabilized-HDCells-Activation-99.txt" with lines title "At time 100", "./250.000000-Stabilized-HDCells-Activation-199.txt" with lines title "At time 200"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "250.000000-Stabilization-firing-rates.eps"
set xlabel "Head direction cells"
set ylabel "Firing rate"
set title ""
plot "./250.000000-Stabilized-HDCells-FiringRate-0.txt" with lines title "At time 0", "./250.000000-Stabilized-HDCells-FiringRate-9.txt" with lines title "At time 10", "./250.000000-Stabilized-HDCells-FiringRate-99.txt" with lines title "At time 100", "./250.000000-Stabilized-HDCells-FiringRate-199.txt" with lines title "At time 200"
set term wxt
set out
set title
set xlabel
set ylabel
