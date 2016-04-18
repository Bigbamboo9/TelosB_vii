@rx_sfd[900];
@rx_clock_drift[900];
@tx_sfd[900];
@tx_clock_drift[900];
@rx_sfd_size[300];
@tx_sfd_size[300];

$is_value;

#$tx_sfd_file = "print1_wo_7samples.out";
#$rx_sfd_file = "print2_wo_7samples.out";
$tx_sfd_file = "print18.out";
$rx_sfd_file = "print28.out";

for ($i = 0; $i < 300; $i++) {
    @rx_sfd_size[$i] = 0;
    @tx_sfd_size[$i] = 0;
    $is_value = 0;
}

$size = 0;

open fin, "<$tx_sfd_file" or die "can not open the file $tx_sfd_file!\n";
open fout, ">tx_sfd_drift.out" or die "can not open the file tx_sfd_drift.out!\n";
while (<fin>) {
	chomp;
	@line = split;

	$time = @line[0];
	$value = @line[1];

	if ($value == 77) {
		$is_value = 1;
		next;
	} elsif ($is_value == 2) {
		$dco_tick = $value;
		$c32khz_tick = @line[2];
		print "$dco_tick $c32khz_tick\n";
		@tx_clock_drift[$min*60+$sec*3+@tx_sfd_size[$min*60+$sec]] = $dco_tick / $c32khz_tick;

		print fout "$size @tx_clock_drift[$min*60+$sec*3+@tx_sfd_size[$min*60+$sec]]\n";
        $size++;

		@tx_sfd_size[$min*60+$sec]++;

	    $is_value = 0;
	    next;
	} elsif ($is_value == 0) {
		next;
	}

	@mark = split(":", $time);
	$min = @mark[1];
	$sec = @mark[2];

	$min = $min - 13;

	printf "$min $sec $value\n";

	@tx_sfd[$min*60+$sec*3+@tx_sfd_size[$min*60+$sec]] = $value;

	$is_value++;
}
close fin;
close fout;

$size = 0;

open fin, "<$rx_sfd_file" or die "can not open the file $rx_sfd_file!\n";
open fout, ">rx_sfd_drift.out" or die "can not open the file rx_sfd_drift.out!\n";
while (<fin>) {
	chomp;
	@line = split;

	$time = @line[0];
	$value = @line[1];

	if ($value == 77) {
		$is_value = 1;
		next;
	} elsif ($is_value == 2) {
		$dco_tick = $value;
		$c32khz_tick = @line[2];
		print "$dco_tick $c32khz_tick\n";
		@rx_clock_drift[$min*60+$sec*3+@rx_sfd_size[$min*60+$sec]] = $dco_tick / $c32khz_tick;

        print fout "$size @rx_clock_drift[$min*60+$sec*3+@rx_sfd_size[$min*60+$sec]]\n";
        $size++;

		@rx_sfd_size[$min*60+$sec]++;

	    $is_value = 0;
	    next;
	} elsif ($is_value == 0) {
		next;
	}

	@mark = split(":", $time);
	$min = @mark[1];
	$sec = @mark[2];

	$min = $min - 13;

	printf "$min $sec $value\n";

	@rx_sfd[$min*60+$sec*3+@rx_sfd_size[$min*60+$sec]] = $value;

	$is_value++;
}
close fin;

$size = 0;
for ($i = 0; $i < 5; $i++) {
	for ($k = 0; $k < 60; $k++) {
		if ( (@rx_sfd_size[$i*60+$k] == @tx_sfd_size[$i*60+$k]) && (@rx_sfd_size[$i*60 + $k] != 0) ) {
			for ($j = 0; $j < @rx_sfd_size[$i*60+$k]; $j++) {
				print "@rx_sfd[$i*60+$k*3+$j] @tx_sfd[$i*60+$k*3+$j]\n";
				@delta[$size] = (@rx_sfd[$i*60+$k*3+$j] / @rx_clock_drift[$i*60+$k*3+$j] - @tx_sfd[$i*60+$k*3+$j] / @tx_clock_drift[$i*60+$k*3+$j]) / 2 * 128;
				print "$i $k @delta[$size]\n";
				$size++;
			}
		}
	}
}

@delta = sort {$a <=> $b} @delta;

open fout, ">delta.out" or die "can not open the file delta.out!\n";
for ($i = 0; $i < $size; $i++) {
	print fout "@delta[$i]\n";
}
close fout;
