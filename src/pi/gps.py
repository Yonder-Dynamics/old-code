"""
Author: Simon Fong
Date: 02/17/2018
Email: simonfong6@gmail.com

This file pulls GPS coordinates from a GPS receiver which ouputs messages in NMEA format. For more information about NMEA GPS formats please visit this link: http://gpsworld.com/what-exactly-is-gps-nmea-data/

The output of NMEA data for latitude and longitude is the following:
(Where 'D' is degrees and 'M' is minutes.)

Latitude:   DDMM.MMMMM
Longitude:  DDDMM.MMMMM

For more information about this format, please visit: https://support.google.com/maps/answer/18539?co=GENIE.Platform%3DDesktop&hl=en

Additonally, we will follow this convention for N,S,E,W.

Lat  N: +
Lat  S: -
Long E: + 
Long W: -

Important Note: GPS Receivers do not work well inside buildings and do take some time to get a reading. Make sure to go outside with direct line of sight with the sky and be prepared to wait up to 5 minutes to get a reading.

"""

import serial
 

class Fake_Serial:
    """ A fake serial object that outputs a line of old data. Only used for 
        testing purposes."""
    def __init__(self):
        pass
        
    def readline(self):
        return "$GPRMC,212544.617,A,3252.9457,N,11714.0749,W,000.0,200.7,110218,,,A*77"

class GPS:
    
    def __init__(self,port='/dev/ttyAMA0',fake=False):
        
        # If fake flag is on, we read fake data.
        if(fake):
            self.ser = Fake_Serial()
        else:
            self.ser = serial.Serial(port=port, baudrate=9600)
        

    def read_lat_long_DMM(self):
        """
        Returns a tuple of strings that represent latitude and longitude in 
        DMM format.
        (latitude,N|S,longitude,E|W)
        """
        
        """
        Line looks like this:
        

$GPRMC,212544.617,A,3252.9457,N,11714.0749,W,000.0,200.7,110218,,,A*77
gps_code,?       ,?,latitude,dir,longitude,dir,?  ,?    ,?     ,,,?
0       ,1       ,2,3       ,4  ,5        ,6  ,7  ,8    ,9     ,10,11,12
        """
        line = self.ser.readline()
        gps_code = line[:6]
	#print(line)
        
        while((gps_code != '$GPGGA') and (gps_code != '$GPGGA')):
            line = self.ser.readline()
            
            gps_code = line[:6]
            #print(gps_code)
        
        
        values = line.split(',')
        
        # Reading lat and long values in DMM format.
        latitude_DMM = values[2]
        latitude_dir = values[3]
        
        longitude_DMM = values[4]
        longitude_dir = values[5]
        
        return (latitude_DMM,latitude_dir,longitude_DMM,longitude_dir)
        
    def read_lat_long_DD(self):
        """
        Returns a tuple of latitude and longitude in DD format with postive values referring to N/E and negative values referring to S/W.
        
        Using this formula:
        (Link: https://www.directionsmag.com/site/latlong-converter/)
        
        Degrees Minutes.m to Decimal Degrees
        .d = M.m / 60
        Decimal Degrees = Degrees + .d
        """
        lat_long_DMM = self.read_lat_long_DMM()
        
        # Parse lat/long values from tuple
        
        lat_DMM = lat_long_DMM[0]   # DDMM.MMMMM
        lat_dir = lat_long_DMM[1]   # N|S
        
        long_DMM = lat_long_DMM[2]  # DDDMM.MMMMM
        long_dir = lat_long_DMM[3]  # E|W
        
        # Convert strings to floats and convert DMM to DD format.
        lat_degrees = float(lat_DMM[:2])
        lat_minutes = float(lat_DMM[2:])
        lat_deg_dec = lat_minutes/60
        lat_degrees+= lat_deg_dec
        
        long_degrees = float(long_DMM[:3])
        long_minutes = float(long_DMM[3:])
        long_deg_dec = long_minutes/60
        long_degrees+= long_deg_dec
        
        # Make values negative if direction is pointing South or West.
        if(lat_dir == 'S'):
            lat_degrees = -lat_degrees
            
        if(long_dir == 'W'):
            long_degrees = -long_degrees
            
        return (lat_degrees,long_degrees)
        
        
        
        
        
        
        
        
                
if(__name__ == '__main__'):
    gps = GPS(fake=True)
    while(True):
        print(gps.read_lat_long_DD())
        
