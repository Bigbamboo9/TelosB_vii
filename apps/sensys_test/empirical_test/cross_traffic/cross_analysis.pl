use List::Util qw(max min sum);

$file = @ARGV[0];
$size = 0;

open fin, "<$file" or die "could not open $file!\n";
while (<fin>) {
	chomp;
	@line = split(",");
	$src = @line[0];
	$rssi = @line[2];

	if ($src >= 20) {
		@avg_rssi[$size] = $rssi;
		#printf "$rssi\n";
		$size++;
	}
}
$prr = $size / 200;
$min = min @avg_rssi;
$max = max @avg_rssi;
$avg = sum @avg_rssi;
$avg = $avg / $size;
printf "$prr $min $max $avg\n";
