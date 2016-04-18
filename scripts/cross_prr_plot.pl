{
	$gnufile = "tmp\/cross_prr_plot.plt";
	open gnuout, ">$gnufile" or die "could not open $gnufile!\n";
	print gnuout "set term post eps color solid enh 'ArialMT' 22\n";
	print gnuout "set output 'diagram\/cross_prr_plot.eps'\n";
	print gnuout "set style data histogram\n";
	print gnuout "set boxwidth 0.9 absolute\n";
	print gnuout "set style histogram clustered gap 1\n";
	print gnuout "set style fill pattern 4 noborder\n";
	print gnuout "set xrange [-1:3]\n";
	print gnuout "set ylabel 'Packet Reception Ratio'\n";
	print gnuout "set yrange [0:1]\n";
	print gnuout "set ytics 0,0.2,1\n";
	print gnuout "set size ratio 0.6\n";
	print gnuout "set key samplen 3 spacing 2 font ',20'\n";
	print gnuout "set key top left\n";
	print gnuout "plot 'cross_prr_plot.txt' using 2:xticlabels(1) title 'DT', '' using 3:xticlabels(1) title 'CE', '' using 4:xticlabels(1) title 'CI'\n";
	
	system "gnuplot tmp\/cross_prr_plot.plt";
}