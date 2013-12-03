set multiplot
set origin 0,0.3
set size 1,0.3
plot "./0000-Master-debug-HD.txt" with lines
set origin 0,0
plot "./0000-Master-debug-FR1.txt" with lines
set origin 0,0.6
plot "./0000-Master-debug-FR2.txt" with lines
pause -1
