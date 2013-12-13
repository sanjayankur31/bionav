set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Stabilization-firing-rates.eps"
set xlabel "Head direction cells"
set ylabel "Firing rate"
set title "Firing rate values during stabilization period"
plot "./Stabilized-HDCells-FiringRate-0.txt" with lines title "At time 0", "./Stabilized-HDCells-FiringRate-9.txt" with lines title "At time 10", "./Stabilized-HDCells-FiringRate-99.txt" with lines title "At time 100", "./Stabilized-HDCells-FiringRate-199.txt" with lines title "At time 200"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Stabilization-activation.eps"
set xlabel "Head direction cells"
set ylabel "Activation"
set title "Activation values during stabilization period"
plot "./Stabilized-HDCells-Activation-0.txt" with lines title "At time 0", "./Stabilized-HDCells-Activation-9.txt" with lines title "At time 10", "./Stabilized-HDCells-Activation-99.txt" with lines title "At time 100", "./Stabilized-HDCells-Activation-199.txt" with lines title "At time 200"
set term wxt
set out
set title
set xlabel
set ylabel
