set term post eps color solid enh 'ArialMT' 22
set output 'diagram/dco_clock_drift.eps'
set ylabel 'Variance of Clock Frequencey Factor'
set xlabel 'Time (4 ms per tick)'
set yrange [0:0.05]
set xrange [0:200]
set key top right
plot 'dco_var1.out' u 1:2 with points pt 4 ps 0.9 title 'Node 1', 'dco_var2.out' u 1:2 with points pt 5 ps 0.9 title 'Node 2'