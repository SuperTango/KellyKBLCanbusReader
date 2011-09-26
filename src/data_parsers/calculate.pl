#!/usr/bin/perl
use strict;
use warnings;

my $diffCol = 1;
my $distanceCol = 11;
my $battVoltCol = 13;
my $battCurrCol = 15;

my $file = $ARGV[0];
open ( my $fh, $file ) || die;
my $totalWh = 0;
my $totalTime = 0;
my $totalDistance = 0;
while ( my $line = <$fh> ) {
    chomp $line;
    next if ( $line =~ /^\s*#/ );
    next if ( $line =~ /^\s*$/ );
    my @arr = split ( ',', $line );
    my $diffMillis = $arr[$diffCol];
    my $distance = $arr[$distanceCol];
    my $bV = $arr[$battVoltCol];
    my $bI = $arr[$battCurrCol];
    my $Wh = $bV * $bI * $diffMillis / 3600000;
    $totalWh += $Wh;
    $totalTime += $diffMillis;
    $totalDistance += $distance;
}
my $WhPerM = $totalWh / $totalDistance;
my $mPerkWh = $totalDistance / $totalWh * 1000;
my $totalHours = int ( $totalTime / 60000 );
my $totalSeconds = ( $totalTime % 60000 ) / 1000;
my $avgMPH = $totalDistance / $totalTime * 3600000;
print "Total Time:      $totalTime\n";
print "Total Time:      $totalHours:$totalSeconds\n";
print "Avg MPH:         $avgMPH\n";
print "Total Wh:        $totalWh\n";
print "Total Distance:  $totalDistance\n";
print "Wh/m:            $WhPerM\n";
print "m/kWh:           $mPerkWh\n";
