#    __ _____       ___  ___ ___  __ __  _       ____  ______   ___   ____  
#   /  ] ___/      /  _]|   |   ||  |  || |     /    ||      | /   \ |    \ 
#  /  (   \_      /  [_ | _   _ ||  |  || |    |  o  ||      ||     ||  D  )
# /  / \__  |    |    _]|  \_/  ||  |  || |___ |     ||_|  |_||  O  ||    / 
#/   \_/  \ |    |   [_ |   |   ||  :  ||     ||  _  |  |  |  |     ||    \ 
#\     \    |    |     ||   |   ||     ||     ||  |  |  |  |  |     ||  .  \
# \____|\___|    |_____||___|___| \__,_||_____||__|__|  |__|   \___/ |__|\_|
import serial
import time
import sys
import os
import math

class CS9Emulator:

    ##############################################################################
    def __init__(self, device, bauds):

        #Technical stuff
        self.HEADER = '\033[95m'
        self.OKBLUE = '\033[94m'
        self.OKGREEN = '\033[92m'
        self.FAIL = '\033[91m'
        self.ENDC = '\033[0m'
        
        #Information for the client
        self.device = device
        self.bauds = bauds

        #Show off
        self.showBanner()

        #Create connection
        self.serial = serial.Serial(self.device, self.bauds, timeout=1)
        self.serial.open()

        self.printLog('Connection established')

        #Do the handshake
        if not self.handshake():
            self.exit()
        
    ##############################################################################
    def stop(self):
        self.sendMessage('STOP')
        self.exit()

    ##############################################################################
    def handshake(self):
        
        self.printLog('Starting handshake')
        data = self.getMessage()
        if data == 'HI SERVER':
            self.printLog('Server says ' + data)
            self.sendMessage('HI CLIENT')
            return True
        else:
            self.printError('Server Handshake response is not valid')
            self.exit()
    ##############################################################################

    ##############################################################################
    def printLog(self, text):

        print(self.OKGREEN + '[Log] ' + text + self.ENDC)
    ##############################################################################

    ##############################################################################
    def printError(self, text):

        print(self.FAIL + '[Error] ' + text + self.ENDC)
    ##############################################################################

    ##############################################################################
    def printCom(self, text):

        print(self.OKBLUE + text + self.ENDC)
    ##############################################################################

    ##############################################################################
    def exit(self):
        self.printLog('Closing connection')
        self.serial.close()
        sys.exit()
    ##############################################################################

    ##############################################################################
    def showBanner(self):

        print( self.HEADER)
        
        print('    __ _____       ___  ___ ___  __ __  _       ____  ______   ___   ____  ')
        print('   /  ] ___/      /  _]|   |   ||  |  || |     /    ||      | /   \\ |    \\ ')
        print('  /  (   \\_      /  [_ | _   _ ||  |  || |    |  o  ||      ||     ||  D  )')
        print(' /  / \\__  |    |    _]|  \_/  ||  |  || |___ |     ||_|  |_||  O  ||    / ')
        print('/   \\_/  \\ |    |   [_ |   |   ||  :  ||     ||  _  |  |  |  |     ||    \\ ')
        print('\\     \\    |    |     ||   |   ||     ||     ||  |  |  |  |  |     ||  .  \\')
        print(' \\____|\\___|    |_____||___|___| \\__,_||_____||__|__|  |__|   \\___/ |__|\\_|')
        print( self.ENDC)
        print( '\n\n')
    ##############################################################################

    ##############################################################################
    def getMessage(self):

        counter = 0
        text = ''
        while True:
            msg = self.serial.rad(512)
            text = text + msg.decode()
            counter = counter + len(msg)
            if counter == 512:
                break
        return text[0:text.find('XXXXX')]
    #############################################################################

    ##############################################################################
    def sendMessage(self, msg):

        if len(msg) >= 512-5:
            return False 
        for i in range(len(msg), 512):
            msg += 'X'
        self.serial.write(msg.encode())
        return True
    ##############################################################################

  
