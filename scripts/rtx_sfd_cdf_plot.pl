#use Statistics::Basic qw(:all);

$file = @ARGV[0];
$cdf_file = "tmp\/sfd_cdf.out";
$size = 0;

open fin, "<$file" or die "could not open $file!\n";
while (<fin>) {
    @mark = split;
    @list[$size] = @mark[0];
    printf "@list[$size]\n";
    $size++;
}
close fin;

#$var = variance(@list);
#$avg = mean(@list);
#print "$avg $var\n"

open fout, ">$cdf_file" or die "could not open $cdf_file!\n";

print fout "@list[0] 0\n";

$cdf_count = 0;
$step_tick = (@list[$size-1] - @list[0]) / 100;
$cdf_delta = @list[0] + $step_tick;

while ($cdf_count < $size) {
    while ( (@list[$cdf_count] < $cdf_delta) and ($cdf_count < $size) ) {
        $cdf_count++;
    }
    $percentage = $cdf_count / $size;
    print fout "$cdf_delta $percentage\n";
    $cdf_delta += $step_tick;
}

{
	$gnufile = "tmp\/rtx_sfd_delta.plt";
	open gnuout, ">$gnufile" or die "could not open $gnufile!\n";
	print gnuout "set term post eps color solid enh 'ArialMT' 22\n";
	print gnuout "set output 'diagram\/rtx_sfd_delta.eps'\n";
	print gnuout "set xrange [@list[0]:@list[$size-1]]\n";
	print gnuout "set ylabel 'CDF'\n";
    print gnuout "set xlabel 'Time (1/4194304 s per tick)'\n";
	print gnuout "set yrange [0:1]\n";
	#print gnuout "set size ratio 0.25\n";
    #print gnuout "set key bottom right\n";
	print gnuout "set key off\n";
	#print gnuout "set key samplen 2 spacing 2 font ',15' box lw 2\n";
	print gnuout "plot 'tmp\/sfd_cdf.out' u 1:2 with linespoints pt 4 ps 0.9 lw 4, ";
	close gnuout;

	system "gnuplot tmp\/rtx_sfd_delta.plt";
}
