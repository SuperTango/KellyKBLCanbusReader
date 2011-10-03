#!/usr/bin/perl
use strict;
use warnings;

my $headers = {
    v1 => [ 'current_millis','diff_millis','count','iterations','date','time','speed_gps','speed_rpm','lat','lon','heading','distance_gps','distance_rpm','batt_voltage','batt_current_reading','batt_current','motor_current','motor_voltage','motor_watts','wh/m_gps','wh/m_rpm','m/kwh_gps','m/kwh_rpm','motor_temp_calc','motor_thermistor_reading','brake_a/d','tps_a/d','controller_power','5v_power','b+','ia','ib','ic','va','vb','vc','pwm','enable_motor_rotation','motor_temp','controller_temp','high_mosfet','low_mosfet','rpm_high','rpm_low','current_%','error_high','error_low'],
    v2 => [ 'current_millis','diff_millis','count','iterations','date','time','fix_age','speed_gps','speed_rpm','lat','lon','altitude','heading','distance_gps','distance_rpm','batt_voltage','batt_current_reading','batt_current','motor_current','motor_voltage','motor_watts','wh/m_gps','wh/m_rpm','m/kwh_gps','m/kwh_rpm','motor_temp_calc','motor_thermistor_reading','brake_a/d','tps_a/d','controller_power','5v_power','b+','ia','ib','ic','va','vb','vc','pwm','enable_motor_rotation','motor_temp','controller_temp','high_mosfet','low_mosfet','rpm_high','rpm_low','current_%','error_high','error_low'],
    v3 => [ 'current_millis','diff_millis','count','iterations','date','time','fix_age','speed_gps','speed_rpm','lat','lon','altitude','heading','failed_cs','distance_gps','distance_rpm','batt_voltage','batt_current_reading','batt_current','motor_current','motor_voltage','motor_watts','wh/m_gps','wh/m_rpm','m/kwh_gps','m/kwh_rpm','motor_temp_calc','motor_thermistor_reading','brake_a/d','tps_a/d','controller_power','5v_power','b+','ia','ib','ic','va','vb','vc','pwm','enable_motor_rotation','motor_temp','controller_temp','high_mosfet','low_mosfet','rpm_high','rpm_low','current_%','error_high','error_low'],
}
my $diffCol = 1;
my $distanceCol = 11;
my $battVoltCol = 13;
my $battCurrCol = 15;

my $file = $ARGV[0];
open ( my $fh, $file ) || die;
my $totalWh = 0;
my $totalTime = 0;
my $totalDistance = 0;
my $type = 'v1';

while ( my $line = <$fh> ) {
    if ( $line =~ /LOGFMT (\d+)/ ) {
        $type = 'v' . $1;
    }
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
