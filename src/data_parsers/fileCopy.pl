#!/usr/bin/perl
use strict;
use warnings;

my $srcDir = "/Volumes/NO NAME";
my $destDir = "/Volumes/Data/Users/altitude/src/Arduino/KellyKBLCanbusReader/nogit/LogData";

opendir ( DIR, $srcDir ) || die;
foreach my $file ( sort ( readdir ( DIR ) ) ) {
    next if ( $file !~ /^\d{5}-(LG|RW|NM)\.(CSV|GPS)$/ );
    my $type = $1;
    my $fullFile = $srcDir . '/' . $file;
    my ($dev,$ino,$mode,$nlink,$uid,$gid,$rdev,$size, $atime,$mtime,$ctime,$blksize,$blocks) = stat($fullFile);
    my ($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) = localtime($mtime);
    my $dateStr = sprintf ( "%02d_%02d_%02d-%02d_%02d_%02d", ( $year + 1900 ), ( $mon + 1 ), $mday, $hour, $min, $sec );
    my $newDir = sprintf ( "%s/%02d_%02d_%02d", $destDir, ( $year + 1900 ), ( $mon + 1 ), $mday );
    if ( ! -d $newDir ) {
        mkdir ( $newDir ) || die "Failed making directory $newDir: $!";
    }

    my $newFile;
    if ( $type eq 'LG' ) {
        $newFile = $dateStr . '-log.CSV';
    } elsif ( $type eq 'RW' ) {
        $newFile = $dateStr . '-raw.CSV';
    } elsif ( $type eq 'NM' ) {
        $newFile = $dateStr . '-gps.nmea';
    } else {
        next;
    }
    my $newFullFile = $newDir . '/' . $newFile;
    print "$file -> $newFile: ";
    if ( -f $newFullFile ) {
        print "File Exists. Skipping.\n";
        next;
    }

    print "Copying...";
    open ( SRC, $fullFile ) || die "Couldn't open new file $fullFile for reading: $!";
    open ( DEST, '>', $newFullFile ) || die "Couldn't open new file $newFile for writing: $!";
    while ( my $line = <SRC> ) {
        print DEST $line;
    }
    close ( DEST );
    utime ( $mtime, $mtime, $newFullFile );
    print "Done\n";
}
