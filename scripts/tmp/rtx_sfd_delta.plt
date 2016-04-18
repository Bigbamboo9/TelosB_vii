set term post eps color solid enh 'ArialMT' 22
set output 'diagram/rtx_sfd_delta.eps'
set xrange [14.6502318577461:18.3221409555837]
set ylabel 'CDF'
set xlabel 'Time (1/4194304 s per tick)'
set yrange [0:1]
set key off
plot 'tmp/sfd_cdf.out' u 1:2 with linespoints pt 4 ps 0.9 lw 4, 