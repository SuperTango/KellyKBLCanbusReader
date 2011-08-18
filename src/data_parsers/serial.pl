# Sample Perl script to transmit number
# to Arduino then listen for the Arduino
# to echo it back

use Device::SerialPort;

# Set up the serial port
# 19200, 81N on the USB ftdi driver
my $port = Device::SerialPort->new("/dev/tty.usbmodem1a21");
$port->databits(8);
$port->baudrate(115200);
$port->parity("none");
$port->stopbits(1);

my $count = 0;
while (1) {
    # Poll to see if any data is coming in
    my $char = $port->lookfor();

    # If we get data, then print it
    # Send a number to the arduino
    if ($char) {
        print "Recieved character: " . $char . " \n";
    } else {
        sleep(1);
        $count++;
        #my $count_out = $port->write("$count\n");
        #print "Sent     character: $count \n";
    }
}

