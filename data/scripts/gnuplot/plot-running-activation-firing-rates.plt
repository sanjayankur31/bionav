set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Running-firing-rates.eps"
set xlabel "Head direction cells"
set ylabel "Firing rate"
set title "Firing rate values"
plot "./Running-HDCells-FiringRate-100.txt" with lines title "At time 10", "./Running-HDCells-FiringRate-5000.txt" with lines title "At time 500", "./Running-HDCells-FiringRate-10000.txt" with lines title "At time 1000", "./Running-HDCells-FiringRate-15000.txt" with lines title "At time 1500"
set term wxt
set out
set terminal postscript eps enhanced color font "Helvetica,20" linewidth 2 
set output "Running-activation.eps"
set xlabel "Head direction cells"
set ylabel "Activation"
set title "Activation values"
plot "./Running-HDCells-Activation-100.txt" with lines title "At time 10", "./Running-HDCells-Activation-5000.txt" with lines title "At time 500", "./Running-HDCells-Activation-10000.txt" with lines title "At time 1000", "./Running-HDCells-Activation-15000.txt" with lines title "At time 1500"
set term wxt
set out
set title
set xlabel
set ylabel

