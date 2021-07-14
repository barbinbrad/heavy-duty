from conversions import Conversions
from math import atan
import re

"""
Usage:
from nmea import NmeaParser

sentence = '$GPGGA,115739.00,4158.8441367,N,09147.4416929,W,4,13,0.9,255.747,M,-32.00,M,01,0000*6E'

gps = NmeaParser()
gps.parse_sentence(sentence)

print(gps.latitude, gps.longitude)

"""

class NmeaParser:
    def __init__ (self, local_utc_offset=-4): 
        
        # Time
        self.timestamp = [0, 0, 0]
        self.local_utc_offset = local_utc_offset
        self.last_updated_utc = 0.0

        # Position/Motion
        self.latitude = 0.0
        self.longitude = 0.0
        self.speed = 0.0 # Kilometers/hour
        self.course = 0.0
        self.course_dual = 0.0
        self.altitude = 0.0
        self.geoid_height = 0.0
        self.roll = 0.0

        # GPS Info
        self.satellites_in_use = 0
        self.hdop_x_100 = 0
        self.pdop_x_100 = 0
        self.vdop_x_100 = 0
        self.valid = False
        self.fix_stat = 0
        
        # Kalman Filter for Roll
        self.Pc = 1.0
        self.G = 1.0
        self.Xp = 1.0 
        self.Zp = 1.0 
        self.Xe_roll = 1.0 
        self.P = 1.0    
        
        # Constants
        self.SENTENCE_MAX_LENGTH = 120
        self.HEMISPHERES = ('N', 'S', 'E', 'W')     
        self.VAR_ROLL = 0.1
        self.VAR_PROCESS = 0.0003

    @property
    def fix_description(self):
        it = self.fix_stat
        if (it == 0):
            return 'Invalid'
        elif (it == 1):
            return 'GPS single'
        elif (it == 2):
            return 'DGPS'
        elif (it == 3):
            return 'PPS'
        elif (it == 4):
            return 'RTK Fix coordinate'
        elif (it == 5):
            return 'RTK Float'
        elif (it == 6):
            return 'Estimate'
        elif (it == 7):
            return 'Manual Input'
        elif (it == 8):
            return 'Simulation'
        else:
            return 'Unkown'

    def parse_sentence(self, sentence):
        # Garmin bug fix
        if(len(sentence) > self.SENTENCE_MAX_LENGTH):
            return

        words = sentence.split(',')
        identifier = words[0]

        if(identifier == '$GPGGA' or identifier == '$GNGGA'):
            if len(words) > 13:
                success = self.parse_gga(words)
        
        if(identifier == '$GPVTG' or identifier == '$GNTVG'):
            if len(words) > 7:
                success = self.parse_vtg(words)
        
        if(identifier == '$GPHPD'):
            success = self.parse_hpd(words)

        if(identifier == '$PAOGI'):
            success = self.parse_ogi(words)

        if(identifier == '$GPHDT' or identifier == '$GNHDT'):
            success = self.parse_hdt(words)

        if(identifier == '$PTNL'):
            if len(words) > 8:
                success = self.parse_avr(words)
        
        if(identifier == '$GNTRA'):
            success = self.parse_tra(words)

        if(identifier == '$PSTI' and words[1] == '032'):
            success = self.parse_sti032(words)

        if(success):
            self.log_success(identifier)
        else:
            if(not self.is_empty_or_blank(identifier)):
                self.log_error(words)

    def parse_gga(self, words):

        try:
            utc_string = words[1]
            
            if utc_string:
                hours = (int(utc_string[0:2]) + self.local_utc_offset) % 24
                minutes = int(utc_string[2:4])
                seconds = float(utc_string[4:])
                self.last_updated_utc = float(utc_string)
            else:
                hours = 0
                minutes = 0
                seconds = 0.0

            fix_status = int(words[6])
            satelittes_in_use = int(words[7])
        except:
            return False
   
        try:
            hdop = int(float(words[8])*100)
        except:
            hdop = 0

        if fix_status:
            try:
                latitude_string = words[2]
                latitude = int(latitude_string[0:2])
                latitude -= 2
                lat_minutes = float(latitude_string[2:])
                latitude += lat_minutes * 0.01666666666667
                lat_hemi = words[3]

                longitude_string = words[4]
                longitude = int(longitude_string[0:3])
                longitude -= 2
                lon_minutes = float(longitude_string[3:])
                longitude += lon_minutes * 0.01666666666667
                lon_hemi = words[5]

            except ValueError:
                return False

            if lat_hemi not in self.HEMISPHERES:
                return False
            elif lat_hemi == 'S':
                latitude = latitude * -1
            
            if lon_hemi not in self.HEMISPHERES:
                return False
            elif lon_hemi == 'W':
                longitude = longitude * -1
                
            try:
                altitude = float(words[9])
                geoid_height = float(words[11])
            except ValueError:
                altitude = 0
                geoid_height = 0

            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude
            self.geoid_height = geoid_height

        self.timestamp = [hours, minutes, seconds]
        self.fix_stat = fix_status if fix_status <= 8 else 0
        self.satellites_in_use = satelittes_in_use
        self.hdop_x_100 = hdop
        
        return True
        

    def parse_vtg(self, words):
        try:
            course = float(words[1])
            kmph = float(words[7])
        except ValueError:
            return False

        self.speed = kmph
        self.course = course
        return True

    def parse_hpd(self, words):
        try:
            course_dual = float(words[3])
            pitch = float(words[4])
            baseline = float(words[18])
        except ValueError:
            return False

        self.course_dual = course_dual
        self.roll = pitch if baseline > 0 else 0

        return True

    def parse_ogi(self, words):
        try:
            utc_string = words[1]

            if utc_string:
                hours = (int(utc_string[0:2]) + self.local_utc_offset) % 24
                minutes = int(utc_string[2:4])
                seconds = float(utc_string[4:])
                self.last_updated_utc = float(utc_string)
            else:
                hours = 0
                minutes = 0
                seconds = 0.0

            fix_status = int(words[6])
            satelittes_in_use = int(words[7])
        except:
            return False
   
        try:
            hdop = int(float(words[8])*100)
        except:
            hdop = 0

        if fix_status:
            try:
                latitude_string = words[2]
                latitude = int(latitude_string[0:2])
                latitude -= 2
                lat_minutes = float(latitude_string[2:])
                latitude += lat_minutes * 0.01666666666667
                lat_hemi = words[3]

                longitude_string = words[4]
                longitude = int(longitude_string[0:3])
                longitude -= 2
                lon_minutes = float(longitude_string[3:])
                longitude += lon_minutes * 0.01666666666667
                lon_hemi = words[5]

            except ValueError:
                return False

            if lat_hemi not in self.HEMISPHERES:
                return False
            elif lat_hemi == 'S':
                latitude = latitude * -1
            
            if lon_hemi not in self.HEMISPHERES:
                return False
            elif lon_hemi == 'W':
                longitude = longitude * -1
                
            try:
                altitude = float(words[9])
                kmh = Conversions.KNOT_TO_KPH * float(words[11]) 
                roll = float(words[13])                
            except ValueError:
                altitude = 0
                kmh = 0
                roll = 0

            try:
                course = float(words[12])
                course_dual = float(words[13])
            except ValueError:
                course = 0
                course_dual = 0

            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude
            self.speed = kmh
            self.course = course
            self.course_dual = course_dual

        self.timestamp = [hours, minutes, seconds]
        self.fix_stat = fix_status if fix_status <= 8 else 0
        self.satellites_in_use = satelittes_in_use
        self.hdop_x_100 = hdop
        
        return True

    def parse_hdt(self, words):
        try:
            course_dual = float(words[1])
        except ValueError:
            return False

        self.course_dual = course_dual
        return True

    def parse_avr(self, words):
        try:
            roll = float(words[7]) if words[8] == 'Roll' else float(words[5])
            roll = self.kalman_filter(roll)
        except ValueError:
            return False

        self.roll = roll
        return True

    def parse_tra(self, words):
        try:
            course_dual = float(words[2])
            roll = float(words[3])
            transolution = int(words[5])
        except ValueError:
            return False

        self.course_dual = course_dual
        self.roll = 0 if transolution != 4 else roll
        return True

    def parse_sti032(self, words):
        try:
            baseline_angle = float(words[10])
            course_dual = baseline_angle + 90.0 if baseline_angle < 270.0 else baseline_angle - 270.0 # rover on left, base on right
        except ValueError:
            return False

        try:
            height_difference = float(words[8]) # rover height - base height
            length_difference = float(words[9]) # distance between base and rover
            radians = atan(height_difference / length_difference)
            roll = Conversions.RAD_TO_DEG * radians
            roll = self.kalman_filter(roll)
        except ValueError:
            return False

        self.course_dual = course_dual
        self.roll = roll
        return True
            
    def kalman_filter(self, roll):
        # Kalman Filter
        self.Pc = self.P + self.VAR_PROCESS
        self.G = self.Pc / (self.Pc + self.VAR_ROLL)
        self.P = (1 - self.G) * self.Pc
        self.Xp = self.Xe_roll
        self.Zp = self.Xp
        self.Xe_roll = ( self.G * ( roll - self.Zp )) + self.Xp

        return self.Xe_roll

    @staticmethod
    def is_empty_or_blank(s):
        """ This function checks if given string is empty
        or contain only shite spaces"""
        return re.search("^\s*$", s)

    def log_success(self, identifier):
        print('[SUCCESS]    {id}'.format( id = identifier))
        self.valid = True
    
    def log_error(self, words):
        self.valid = False
        print('[ERROR]      {sentence}'.format( sentence = words.join(',')))