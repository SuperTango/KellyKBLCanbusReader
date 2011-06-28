#!/usr/bin/perl
use strict;
use warnings;

open (A , "../../nogit/LogData/00045-LG.CSV" )||die;
my $last = '';
while ( my $line = <A> ) {
    my ( $time, $line ) = split ( /,/, $line, 2 );
    print "$line";
}
