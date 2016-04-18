$file = @ARGV[0];
$size = 0;
@factor[4];
$index = 0;

open fin, "<$file" or die "could not open $file!\n";
open fout, ">dco_var.out" or die "could not open drift_factor.out!\n";
while (<fin>) {
    chomp;
	@line = split;

	@factor[$index] = @line[1];
	$index++;

	if ($index == 16) {
		@factor = sort {$a <=> $b} @factor;
		#for ($i = 0; $i < 4; $i++) {
		#	printf "@factor[$i] ";
		#}
		#printf "\n";
		$diff = @factor[3] - @factor[0];
        print fout "$size $diff\n";
        $size++;
        $index = 0;
	}
}
close fin;
close fout;