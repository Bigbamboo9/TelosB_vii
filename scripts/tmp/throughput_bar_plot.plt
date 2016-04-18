set term post eps color solid enh 'ArialMT' 22
set output 'diagram/throughput_bar_plot.eps'
set style data histogram
set boxwidth 0.9 absolute
set style histogram clustered gap 1
set style fill pattern 4 noborder
set xrange [-1:2]
set ylabel 'Throughput(KB/s)'
set yrange [0:24]
set ytics 0,4,22
set size ratio 1
plot 'throughput_bar.txt' using 2:xticlabels(1) notitle, 21.68 with lines notitle lw 4
