set terminal postscript eps enhanced color font "Helvetica,15" linewidth 1 
set output "Firing rates and HD.eps"
set title ""
set multiplot
set xlabel ""
set ylabel "Firing rate"
set origin 0,0.6
set size 1,0.3
plot "./0000-Master-debug-FR1.txt" with lines title "Rotation Cell 1"
set xlabel ""
set ylabel "Firing rate"
set origin 0,0.3
plot "./0000-Master-debug-FR2.txt" with lines title "Rotation Cell 2"
set origin 0,0
set xlabel "Time"
set ylabel "Head direction"
set ytics 0,45
plot "./0000-Master-debug-HD.txt" with lines title "Head direction"

set xlabel
set ylabel
unset multiplot
set term wxt
set out
