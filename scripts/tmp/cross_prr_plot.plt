set term post eps color solid enh 'ArialMT' 22
set output 'diagram/cross_prr_plot.eps'
set style data histogram
set boxwidth 0.9 absolute
set style histogram clustered gap 1
set style fill pattern 4 noborder
set xrange [-1:3]
set ylabel 'Packet Reception Ratio'
set yrange [0:1]
set ytics 0,0.2,1
set size ratio 0.6
set key samplen 3 spacing 2 font ',20'
set key top left
plot 'cross_prr_plot.txt' using 2:xticlabels(1) title 'DT', '' using 3:xticlabels(1) title 'CE', '' using 4:xticlabels(1) title 'CI'
