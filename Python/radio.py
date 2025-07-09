from machine import Pin, I2C
import time
from rotary_irq_rp2 import RotaryIRQ

class Radio:
    
    def __init__( self, NewFrequency, NewVolume, NewMute ):

#
# set the initial values of the radio
#
        self.Volume = 2
        self.Frequency = 88
        self.Mute = False
#
# Update the values with the ones passed in the initialization code
#
        self.SetVolume( NewVolume )
        self.SetFrequency( NewFrequency )
        self.SetMute( NewMute )
        
        
# Initialize I/O pins associated with the radio's I2C interface

        self.i2c_sda = Pin(26)
        self.i2c_scl = Pin(27)

#
# I2C Device ID can be 0 or 1. It must match the wiring. 
#
# The radio is connected to device number 1 of the I2C device
#
        self.i2c_device = 1 
        self.i2c_device_address = 0x10

#
# Array used to configure the radio
#
        self.Settings = bytearray( 8 )


        self.radio_i2c = I2C( self.i2c_device, scl=self.i2c_scl, sda=self.i2c_sda, freq=200000)
        self.ProgramRadio()

    def SetVolume( self, NewVolume ):
#
# Conver t the string into a integer
#
        try:
            NewVolume = int( NewVolume )
            
        except:
            return( False )
        
#
# Validate the type and range check the volume
#
        if ( not isinstance( NewVolume, int )):
            return( False )
        
        if (( NewVolume < 0 ) or ( NewVolume >= 16 )):
            return( False )

        self.Volume = NewVolume
        return( True )



    def SetFrequency( self, NewFrequency ):
#
# Convert the string into a floating point value
#
        try:
            NewFrequency = float( NewFrequency )
            
        except:
            return( False )
#
# validate the type and range check the frequency
#
        if ( not ( isinstance( NewFrequency, float ))):
            return( False )
    
        if (( NewFrequency < 88.0 ) or ( NewFrequency > 108.0 )):
            return( False )

        self.Frequency = NewFrequency
        return( True )
        
    def SetMute( self, NewMute ):
        
        try:
            self.Mute = bool( int( NewMute ))
            
        except:
            return( False )
        
        return( True )

#
# convert the frequency to 10 bit value for the radio chip
#
    def ComputeChannelSetting( self, Frequency ):
        Frequency = int( Frequency * 10 ) - 870
        
        ByteCode = bytearray( 2 )
#
# split the 10 bits into 2 bytes
#
        ByteCode[0] = ( Frequency >> 2 ) & 0xFF
        ByteCode[1] = (( Frequency & 0x03 ) << 6 ) & 0xC0
        return( ByteCode )

#
# Configure the settings array with the mute, frequency and volume settings
#
    def UpdateSettings( self ):
        
        if ( self.Mute ):
            self.Settings[0] = 0x80
        else:
            self.Settings[0] = 0xC0
    
        self.Settings[1] = 0x09 | 0x04
        self.Settings[2:3] = self.ComputeChannelSetting( self.Frequency )
        self.Settings[3] = self.Settings[3] | 0x10
        self.Settings[4] = 0x04
        self.Settings[5] = 0x00
        self.Settings[6] = 0x84
        self.Settings[7] = 0x80 + self.Volume

#        
# Update the settings array and transmitt it to the radio
#
    def ProgramRadio( self ):

        self.UpdateSettings()
        self.radio_i2c.writeto( self.i2c_device_address, self.Settings )

#
# Extract the settings from the radio registers
#
    def GetSettings( self ):
#        
# Need to read the entire register space. This is allow access to the mute and volume settings
# After and address of 255 the 
#
        self.RadioStatus = self.radio_i2c.readfrom( self.i2c_device_address, 256 )

        if (( self.RadioStatus[0xF0] & 0x40 ) != 0x00 ):
            MuteStatus = False
        else:
            MuteStatus = True
            
        VolumeStatus = self.RadioStatus[0xF7] & 0x0F
    
    #
    # Convert the frequency 10 bit count into actual frequency in Mhz
    #
        FrequencyStatus = (( self.RadioStatus[0x00] & 0x03 ) << 8 ) | ( self.RadioStatus[0x01] & 0xFF )
        FrequencyStatus = ( FrequencyStatus * 0.1 ) + 87.0
        
        if (( self.RadioStatus[0x00] & 0x04 ) != 0x00 ):
            StereoStatus = True
        else:
            StereoStatus = False
        
        return( MuteStatus, VolumeStatus, FrequencyStatus, StereoStatus )

#
# initialize the FM radio
#
fm_radio = Radio( 101.9, 2, False )

# Init Buttons
button_sw1 = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_DOWN) #Rotary Encoder Switch 1
button_sw2 = machine.Pin(3, machine.Pin.IN, machine.Pin.PULL_DOWN) #Rotary Encoder Switch 2
button_sw3 = machine.Pin(6, machine.Pin.IN, machine.Pin.PULL_DOWN) #Switch 3
button_sw4 = machine.Pin(7, machine.Pin.IN, machine.Pin.PULL_DOWN) #Switch 4

#Rotary Encoders
# Initialize the rotary encoder
# pin_num_dt: Data pin, pin_num_clk: Clock pin
# min_val, max_val: Set a range if needed, otherwise use RANGE_UNBOUNDED
# reverse: Set to True if rotation direction is inverted

# For volume control?
rotary1 = RotaryIRQ(pin_num_dt=1, pin_num_clk=2,
                min_val=0, max_val=100, reverse=False,
                range_mode=RotaryIRQ.RANGE_WRAP) # or RotaryIRQ.RANGE_UNBOUNDED

# For frequency control?
rotary2 = RotaryIRQ(pin_num_dt=4, pin_num_clk=5,
            min_val=0, max_val=100, reverse=False,
            range_mode=RotaryIRQ.RANGE_WRAP) # or RotaryIRQ.RANGE_UNBOUNDED

r1_old = rotary1.value()
r2_old = rotary2.value()

# FSM for button checking
prevButton = 0
curButton = 0

while ( True ):

#
# display the menu
#
    print("")
    print( "ECE 299 FM Radio Demo Menu" )
    print("")
    print( "1 - change radio frequency" )
    print( "2 - change volume level" )
    print( "3 - mute audio" )
    print( "4 - read current settings" )
    print( "5 - Rotate to change volume" )

    new_r1 = rotary1.value()
    new_r2 = rotary2.value()

    if new_r1 != r1_old:
        print(f"Rotary1 moved: {new_r1}")
        r1_old = new_r1

    if new_r2 != r2_old:
        print(f"Rotary2 moved: {new_r2}")
        r2_old = new_r2
    
    select = input( "Enter menu number > " )
    


#
# Set radio frequency
#
    if ( select == "1" ):
        Frequency = input( "Enter frequncy in Mhz ( IE 100.3 ) > " )

        if ( fm_radio.SetFrequency( Frequency ) == True ):
            fm_radio.ProgramRadio()
        else:
            print( "Invalid frequency( Range is 88.0 to 108.0 )" )

#
# Set volume level of radio
#
    elif ( select == "2" ):

        Volume = input( "Enter volume level ( 0 to 15, 15 is loud ) > " )
        
        if ( fm_radio.SetVolume( Volume ) == True ):
            fm_radio.ProgramRadio()
        else:
            print( "Invalid volume level( Range is 0 to 15 )" )
        
#        
# Enable mute of radio       
#        
    elif( select == "3" ):
        Mute = input( "Enter mute ( 1 for Mute, 0 for audio ) > " )
        
        if ( fm_radio.SetMute( Mute ) == True ):
            fm_radio.ProgramRadio()
        else:
            print( "Invalid mute setting" )

#
# Display radio current settings
#
    elif( select == "4" ):
        Settings = fm_radio.GetSettings()

        print( Settings )
        print("")
        print("Radio Status")
        print("")

        print( "Mute: ", end="" )
        if ( Settings[0] == True ):
            print( "enabled" )
        else:
            print( "disabled" )

        print( "Volume: %d" % Settings[1] )

        print( "Frequency: %5.1f" % Settings[2] )

        print( "Mode: ", end="" )
        if ( Settings[3] == True ):
            print( "stereo" )
        else:
            print( "mono" )
            
    elif select == "5":
        print("Press CTRL+C to save your volume")
        newVolume = 0

        val_old = rotary1.value()

        while (True):
            try:
                print("Rotate the knob to change volume. Press CTRL+C to save.")
                while True:
                    val_new = rotary1.value()

                    if val_new != val_old:
                        val_old = val_new
                        print("Volume:", val_new)
                        if fm_radio.SetVolume(val_new):
                            fm_radio.ProgramRadio()

                    time.sleep_ms(50)

            except KeyboardInterrupt:
                print("Volume saved as:", val_old)
                input("Press Enter to return to menu...")

    else:
        print( "Invalid menu option" )