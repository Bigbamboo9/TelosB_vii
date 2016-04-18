{
	$gnufile = "tmp\/throughput_bar_plot.plt";
	open gnuout, ">$gnufile" or die "could not open $gnufile!\n";
	print gnuout "set term post eps color solid enh 'ArialMT' 22\n";
	print gnuout "set output 'diagram\/throughput_bar_plot.eps'\n";
	print gnuout "set style data histogram\n";
	print gnuout "set boxwidth 0.9 absolute\n";
	print gnuout "set style histogram clustered gap 1\n";
	print gnuout "set style fill pattern 4 noborder\n";
	print gnuout "set xrange [-1:2]\n";
	print gnuout "set ylabel 'Throughput(KB/s)'\n";
	print gnuout "set yrange [0:24]\n";
	print gnuout "set ytics 0,4,22\n";
	print gnuout "set size ratio 1\n";
	#print gnuout "set key samplen 2 spacing 2 font ',20'\n";
	#print gnuout "set key top left\n";
	print gnuout "plot 'throughput_bar.txt' using 2:xticlabels(1) notitle, 21.68 with lines notitle lw 4\n";
	
	system "gnuplot tmp\/throughput_bar_plot.plt";
}