set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Stabilization-firing-rates-HDCells.eps"
set xlabel "Head direction cells"
set ylabel "Firing rate"
set title ""
plot "./Stabilized-HDCells-FiringRate-0.txt" with lines title "At time 0", "./Stabilized-HDCells-FiringRate-9.txt" with lines title "At time 10", "./Stabilized-HDCells-FiringRate-99.txt" with lines title "At time 100", "./Stabilized-HDCells-FiringRate-199.txt" with lines title "At time 200"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Stabilization-activation-HDCells.eps"
set xlabel "Grid cells"
set ylabel "Activation"
set title ""
plot "./Stabilized-GridCells-Activation-0.txt" with lines title "At time 0", "./Stabilized-GridCells-Activation-9.txt" with lines title "At time 10", "./Stabilized-GridCells-Activation-99.txt" with lines title "At time 100", "./Stabilized-GridCells-Activation-199.txt" with lines title "At time 200"
set term wxt
set out
set output "Stabilization-firing-rates-GridCells.eps"
set xlabel "Grid cells"
set ylabel "Firing rate"
set title ""
plot "./Stabilized-GridCells-FiringRate-0.txt" with lines title "At time 0", "./Stabilized-GridCells-FiringRate-9.txt" with lines title "At time 10", "./Stabilized-GridCells-FiringRate-99.txt" with lines title "At time 100", "./Stabilized-GridCells-FiringRate-199.txt" with lines title "At time 200"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Stabilization-activation-GridCells.eps"
set xlabel "Head direction cells"
set ylabel "Activation"
set title ""
plot "./Stabilized-HDCells-Activation-0.txt" with lines title "At time 0", "./Stabilized-HDCells-Activation-9.txt" with lines title "At time 10", "./Stabilized-HDCells-Activation-99.txt" with lines title "At time 100", "./Stabilized-HDCells-Activation-199.txt" with lines title "At time 200"
set title
set xlabel
set ylabel
