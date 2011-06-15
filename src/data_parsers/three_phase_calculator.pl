#!/usr/bin/perl
# Try to figure out how to parse the threephase (Ia, Ib, Ic, Va, Vb, Vc) data.
use strict;
use warnings;

open (A , "../nogit/LogData/LG_00122b.csv" )||die;
my $line = <A>;
my $last = '';

#my $s1 = "\$GPGSA,A,2,29,19,28,,,,,,,,,,23.4,12.1,20.0";
#my $c1 = checksum ( $s1 );
#print "$s1*$c1\n";exit;
#
my $iMax = 0;
my $iMaxRow = 0;
my $vMax = 0;
my $vMaxRow = 0;
my $row = 0;

while ( $line = <A> ) {
    $row++;
    my @arr = split ( /,/, $line );
    #next if ( int( $arr[0] / 1000 ) != 233 );
    for ( my $i = 16; $i <=18; $i++ ) {
        if ( $arr[$i] > $iMax ) {
            $iMax = $arr[$i];
            $iMaxRow = $row;
        }
        if ( $arr[$i+3] > $vMax ) {
            $vMax = $arr[$i+3];
            $vMaxRow = $row;
        }
    }
    #print $line;
}
print "iMax: $iMax (row: $iMaxRow)\n";
print "vMax: " . ( $vMax / 1.84 ) . "($vMax) (row: $vMaxRow)\n";

sub checksum {
    my ($line) = @_;
    my $csum = 0;
    $csum ^= unpack("C",(substr($line,$_,1))) for(1..length($line)-1);

    #print "Checksum: $csum\n" ;
    return (sprintf("%2.2X",$csum));
}

