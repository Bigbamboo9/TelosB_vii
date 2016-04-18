$file1 = @ARGV[0];
$file2 = @ARGV[1];

{
	$gnufile = "tmp\/dco_clock_drift.plt";
	open gnuout, ">$gnufile" or die "could not open $gnufile!\n";
	print gnuout "set term post eps color solid enh 'ArialMT' 22\n";
	print gnuout "set output 'diagram\/dco_clock_drift.eps'\n";
	print gnuout "set ylabel 'Variance of Clock Frequencey Factor'\n";
    #print gnuout "set xlabel 'Packet Sequence Number'\n";
    print gnuout "set xlabel 'Time (4 ms per tick)'\n";
	print gnuout "set yrange [0:0.05]\n";
	print gnuout "set xrange [0:200]\n";
	#print gnuout "set size ratio 0.25\n";
    print gnuout "set key top right\n";
	#print gnuout "set key off\n";
	#print gnuout "set key samplen 2 spacing 2 font ',15' box lw 2\n";
	print gnuout "plot '$file1' u 1:2 with points pt 4 ps 0.9 title 'Node 1', '$file2' u 1:2 with points pt 5 ps 0.9 title 'Node 2'";
	close gnuout;

	system "gnuplot tmp\/dco_clock_drift.plt";
}