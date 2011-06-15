#!/usr/bin/perl

#
# convert data from a file on the SD card to a NMEA file for import into
# google earth.
#
use strict;
use warnings;

open (A , "../nogit/LogData/LG_00122b.csv" )||die;
my $line = <A>;
my $last = '';

#my $s1 = "\$GPGSA,A,2,29,19,28,,,,,,,,,,23.4,12.1,20.0";
#my $c1 = checksum ( $s1 );
#print "$s1*$c1\n";exit;

while ( $line = <A> ) {
    my @arr = split ( /,/, $line );
    my ( $year, $mon, $day, $hour, $min, $sec ) = $arr[1] =~ /^(..)(.)(..)_(..)(..)(.*)/;
    my $dateStr = sprintf ( "%02d%02d%02d", $day, $mon, $year );
    my $timeStr = sprintf ( "%02d%02d%02d.000", $hour, $min, $sec );
    if ( "$dateStr$timeStr" ne $last ) {
        #print "$dateStr, $timeStr\n";
        $last = "$dateStr$timeStr";
        my $sentence = "\$GPRMC,$timeStr,A,$arr[5],$arr[6],$arr[7],$arr[8],$arr[9],$arr[10],$dateStr,,";
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

