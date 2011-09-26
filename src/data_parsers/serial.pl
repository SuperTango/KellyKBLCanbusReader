# Sample Perl script to transmit number
# to Arduino then listen for the Arduino
# to echo it back

use Device::SerialPort;

# Set up the serial port
# 19200, 81N on the USB ftdi driver
#my $port = Device::SerialPort->new("/dev/tty.usbmodem1a21");
my $port = Device::SerialPort->new("/dev/tty.usbmodem1d11");
$port->baudrate(115200);
$port->databits(8);
$port->parity("none");
$port->stopbits(1);

my $count = 0;
while (1) {
    # Poll to see if any data is coming in
    my $char = $port->lookfor();

    # If we get data, then print it
    # Send a number to the arduino
    if ($char) {
        $char =~ s/^(.*?)\s*$/$1/;
        chomp ( $char );
        chomp ( $char );
        if ( $char eq 'READY' ) {
            #my $count_out =  $port->write ( "L\n" );
            my $count_out =  $port->write ( "00053-LG.CSV\n" );
            print "COUNT_OUT: $count_out\n";
        } else {
            print "Recieved character: '" . $char . "'\n";
        }
    } else {
        print "sleeping\n";
        sleep(1);
            my $count_out =  $port->write ( "G 00050-LG.CSV\n" );
        $count++;
        #my $count_out = $port->write("$count\n");
        #print "Sent     character: $count \n";
    }
}

