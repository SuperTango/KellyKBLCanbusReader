#!/usr/bin/perl

#
# convert data from a file on the SD card to a NMEA file for import into
# google earth.
#
use strict;
use warnings;

my $file = $ARGV[0];
if ( ! $file ) {
    die "must provide file as first arg";
}
if ( ! -f $file ) {
    die "file: '$file' does not exist";
}
my $outFile = $ARGV[1];
if ( ! $outFile ) {
    ( $outFile = $file ) =~ s/csv$/nmea/i;
}
if ( -f $outFile ) {
    #die "output file '$outFile' exists";
}

open (A , $file )||die $!;
my $line = <A>;
my $last = '';

#my $s1 = "\$GPGSA,A,2,29,19,28,,,,,,,,,,23.4,12.1,20.0";
#my $c1 = checksum ( $s1 );
#print "$s1*$c1\n";exit;

my $dateIndex=4;
my $timeIndex = 5;
my $speedIndex = 6;
my $latIndex = 9;
my $lonIndex = 10;
my $distanceIndex = 12;
my $bearingIndex = 11;
$line = <A>;

while ( $line = <A> ) {
    my @arr = split ( /,/, $line );
    #my ( $year, $mon, $day ) = $arr[2] =~ /^(..)(..)(..)/;
    #my ( $hour, $min, $sec ) = $arr[3] =~ /^(..)(..)(..)/;
    #my $dateStr = sprintf ( "%02d%02d%02d", $day, $mon, $year );
    #my $timeStr = sprintf ( "%02d%02d%02d.000", $hour, $min, $sec );
    if ( ( $arr[$timeIndex] ne $last ) && ( $arr[$distanceIndex] > 0 ) ) {
        #print "$dateStr, $timeStr\n";
        $last = $arr[$timeIndex];
        $arr[$dateIndex] =~ /20(..)(..)(..)/;
        my $dateStr = $3 . $2 . $1;
        my $timeStr = $arr[$timeIndex];
        my $knots = sprintf ( "%.2d", ( $arr[$speedIndex] * 0.868976242 ) );
        my $latDeg = int ($arr[$latIndex] );
        my $latMin = ( $arr[$latIndex] - int ($arr[$latIndex] ) ) * 60;
        my $latStr = sprintf ( "%d%0.4f", $latDeg, $latMin );
        my $lonDeg = abs ( int ($arr[$lonIndex] ) ) ;
        my $lonMin = ( abs($arr[$lonIndex] ) - abs( int ($arr[$lonIndex] ) ) ) * 60;
        my $lonStr = sprintf ( "%d%07.04f", $lonDeg, $lonMin );
        my $sentence = "\$GPRMC,$timeStr.000,A,$latStr,N,$lonStr,W,$knots,$arr[$bearingIndex],$arr[$dateIndex],,";
        my $checksum = checksum ( $sentence );
        print "$sentence*$checksum\n";
    }
}

sub checksum {
    my ($line) = @_;
    my $csum = 0;
    $csum ^= unpack("C",(substr($line,$_,1))) for(1..length($line)-1);

    #print "Checksum: $csum\n" ;
    return (sprintf("%2.2X",$csum));
}

