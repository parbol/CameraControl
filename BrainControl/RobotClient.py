#8888888b.   .d88888b.  888888b.    .d88888b. 88888888888       .d8888b.  888      8888888 8888888888 888b    888 88888888888 
#888   Y88b d88P" "Y88b 888  "88b  d88P" "Y88b    888          d88P  Y88b 888        888   888        8888b   888     888     
#888    888 888     888 888  .88P  888     888    888          888    888 888        888   888        88888b  888     888     
#888   d88P 888     888 8888888K.  888     888    888          888        888        888   8888888    888Y88b 888     888     
#8888888P"  888     888 888  "Y88b 888     888    888          888        888        888   888        888 Y88b888     888     
#888 T88b   888     888 888    888 888     888    888          888    888 888        888   888        888  Y88888     888     
#888  T88b  Y88b. .d88P 888   d88P Y88b. .d88P    888          Y88b  d88P 888        888   888        888   Y8888     888     
#888   T88b  "Y88888P"  8888888P"   "Y88888P"     888           "Y8888P"  88888888 8888888 8888888888 888    Y888     888     
import socket
import time
import sys
import os

class RobotClient:

    ##############################################################################
    def __init__(self, robot_IP, robot_PORT, filenameInput):

        #Technical stuff
        self.HEADER = '\033[95m'
        self.OKBLUE = '\033[94m'
        self.OKGREEN = '\033[92m'
        self.FAIL = '\033[91m'
        self.ENDC = '\033[0m'
        
        #Information for the client
        self.robot_IP = robot_IP
        self.robot_PORT = robot_PORT
        self.fileName = filenameInput

        #Messages from the server
        #self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4, 0x02, 0x8c, 0x02, 0xa5, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00]
        self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4]
        self.serverMessage = [0x02, 0x00, 0xff, 0x44, 0x00, 0x14]
        self.endTransfer = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0f, 0x00, 0x02]

        #Messages from the client
        self.clientHi = [0x00, 0x04, 0x04, 0x1a, 0x17, 0x02, 0xe6, 0xdf, 0x00, 0x00, 0x00, 0x00]
        self.sendallSequenceOK = [0xff, 0xff, 0xff, 0xff]
        self.takePicture = [0x00, 0x04, 0x04, 0xde, 0x16, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00]
        self.sendallorder = [0xff, 0x44, 0x44, 0x6a, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00]

        #Show off
        self.showBanner()

        #Create connection
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.robot_IP, self.robot_PORT))
        self.printLog('Connection to: ' + str(robot_IP) + ' using port: ' + str(robot_PORT))

        #Do the handshake
        self.handshake()
        print('i am here')
        

    ##############################################################################
    def takePic(self):

        self.s.sendall(bytearray(self.takePicture))
        f = open(self.fileName,'wb')
        l = self.s.recv(1024)
        while(l):
            print('iam in the client receive')
            f.write(l)
            l = self.s.recv(1024)
            if not l:
                break
        print('finish file transfer')
        f.close()
    ##############################################################################
    

    ##############################################################################
    def handshake(self):
    
        self.s.sendall(bytearray(self.clientHi))
        data = self.s.recv(12)
        if not self.isEqual(data, self.serverHi):
            self.printError('Server Handshake response is not valid')
            self.exit()
        self.sendConfirmation()

    ##############################################################################

    ##############################################################################
    def isEqual(self, data1, data2):

        thedata1 = bytearray(data1)
        thedata2 = bytearray(data2)
        for i in range(0, len(thedata2)):
            if thedata1[i] != thedata2[i]:
                return False
        return True
    ##############################################################################

    ##############################################################################
    def sendConfirmation(self):

        self.s.sendall(bytearray(self.sendallSequenceOK))
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

        self.s.shutdown(socket.SHUT_RDWR)
        self.s.close()
        sys.exit()
    ##############################################################################

    ##############################################################################
    def showBanner(self):

        print( self.HEADER)
        print( '8888888b.   .d88888b.  888888b.    .d88888b. 88888888888       .d8888b.  888      8888888 8888888888 888b    888 88888888888 ')
        print( '888   Y88b d88P" "Y88b 888  "88b  d88P" "Y88b    888          d88P  Y88b 888        888   888        8888b   888     888     ')
        print( '888    888 888     888 888  .88P  888     888    888          888    888 888        888   888        88888b  888     888     ')
        print( '888   d88P 888     888 8888888K.  888     888    888          888        888        888   8888888    888Y88b 888     888     ')
        print( '8888888P"  888     888 888  "Y88b 888     888    888          888        888        888   888        888 Y88b888     888     ')
        print( '888 T88b   888     888 888    888 888     888    888          888    888 888        888   888        888  Y88888     888     ')
        print( '888  T88b  Y88b. .d88P 888   d88P Y88b. .d88P    888          Y88b  d88P 888        888   888        888   Y8888     888     ')
        print( '888   T88b  "Y88888P"  8888888P"   "Y88888P"     888           "Y8888P"  88888888 8888888 8888888888 888    Y888     888     ')
        print( self.ENDC)
        print( '\n\n')
    ##############################################################################

  

