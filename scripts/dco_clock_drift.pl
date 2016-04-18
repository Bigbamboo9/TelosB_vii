$file = "print8.out";
$size = 0;

open fin, "<$file" or die "could not open $file!\n";
open fout, ">drift_factor.out" or die "could not open drift_factor.out!\n";
while (<fin>) {
    chomp;
	@line = split;

	$time = @line[0];
	$dco_tick = @line[1];
	$c32khz_tick = @line[2];

	$drift_factor = $dco_tick / $c32khz_tick;

	@mark = split(":", $time);
	$min = @mark[1];
	$sec = @mark[2];

    print fout "$size $drift_factor\n";
    $size++;
}
close fin;
close fout;
