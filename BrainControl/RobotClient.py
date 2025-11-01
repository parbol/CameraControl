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
    def __init__(self, robot_IP, robot_PORT, filenameInput, filenameOutput):

        #Technical stuff
        self.HEADER = '\033[95m'
        self.OKBLUE = '\033[94m'
        self.OKGREEN = '\033[92m'
        self.FAIL = '\033[91m'
        self.ENDC = '\033[0m'
        
        #Information for the client
        self.robot_IP = robot_IP
        self.robot_PORT = robot_PORT
        self.filenameInput = filenameInput
        self.filenameOutput = filenameOutput

        #Messages from the server
        #self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4, 0x02, 0x8c, 0x02, 0xa5, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00]
        self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4]
        self.serverMessage = [0x02, 0x00, 0xff, 0x44, 0x00, 0x14]
        self.countingAnswer = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0f, 0x00, 0x02]

        #Messages from the client
        self.clientHi = [0x00, 0x04, 0x04, 0x1a, 0x17, 0x02, 0xe6, 0xdf, 0x00, 0x00, 0x00, 0x00]
        self.sendallSequenceOK = [0xff, 0xff, 0xff, 0xff]
        self.startCounting = [0x00, 0x04, 0x04, 0xde, 0x16, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00]
        self.sendallorder = [0xff, 0x44, 0x44, 0x6a, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00]

        #Show off
        self.showBanner()

        #Create connection
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.robot_IP, self.robot_PORT))
        self.printLog('Connection to: ' + str(robot_IP) + ' using port: ' + str(robot_PORT))

        #Initialize the counter 
        self.counter = list(self.startCounting) 

        #Do the handshake
        self.handshake()

        while True:

            command = self.getCommand()
            if command == 'none':
                if not self.sendallCounter():
                    self.printError('There was a problem with the counter')
                    self.exit()
            else:
                self.processCommand(command)
              
        self.s.close()


    ##############################################################################

    ##############################################################################
    def sendallCounter(self):
        thecounter = bytearray(self.counter)
        a = self.s.sendall(thecounter)
        data = self.s.recv(10)
        thedata = bytearray(data)
        if thedata[9] == thecounter[7] and thedata[8] == thecounter[6]:
            self.sendConfirmation()
            self.updateCrono() 
            return True
        return False
    ##############################################################################
    
    ##############################################################################
    def getFullPackage(self):
        
        header = self.s.recv(8)
        thesize = ord(header[6]) * 16 + ord(header[7])
        data = self.s.recv(thesize)
        return header, data
    ##############################################################################

    ##############################################################################
    def handshake(self):
    
        self.s.sendall(bytearray(self.clientHi))
        header, data = self.getFullPackage()
        if not self.isEqual(header, self.serverHi):
            self.printError('Server Handshake response is not valid')
            self.exit()
        self.sendConfirmation()
        
        header, data = self.getFullPackage()
        if not self.isEqual(header, self.serverMessage):
            self.printError('The server sent an invalid message')
            self.exit()
        self.printLog('Message received from server:')
        self.printCom(data)
        self.sendConfirmation()
    
        header, data = self.getFullPackage()
        if not self.isEqual(header, self.serverMessage):
            self.printError('The server sent an invalid message')
            self.exit()
        self.printLog('Message received from server:')
        self.printCom(data)
        self.sendConfirmation()
        self.printLog('Successful handshake performed')

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

        print self.OKGREEN + '[Log] ' + text + self.ENDC
    ##############################################################################

    ##############################################################################
    def printError(self, text):

        print self.FAIL + '[Error] ' + text + self.ENDC
    ##############################################################################

    ##############################################################################
    def printCom(self, text):

        print self.OKBLUE + text + self.ENDC
    ##############################################################################

    ##############################################################################
    def exit(self):

        self.s.shutdown(socket.SHUT_RDWR)
        self.s.close()
        sys.exit()
    ##############################################################################

    ##############################################################################
    def showBanner(self):

        print self.HEADER
        print '8888888b.   .d88888b.  888888b.    .d88888b. 88888888888       .d8888b.  888      8888888 8888888888 888b    888 88888888888 '
        print '888   Y88b d88P" "Y88b 888  "88b  d88P" "Y88b    888          d88P  Y88b 888        888   888        8888b   888     888     '
        print '888    888 888     888 888  .88P  888     888    888          888    888 888        888   888        88888b  888     888     '
        print '888   d88P 888     888 8888888K.  888     888    888          888        888        888   8888888    888Y88b 888     888     '
        print '8888888P"  888     888 888  "Y88b 888     888    888          888        888        888   888        888 Y88b888     888     '
        print '888 T88b   888     888 888    888 888     888    888          888    888 888        888   888        888  Y88888     888     '
        print '888  T88b  Y88b. .d88P 888   d88P Y88b. .d88P    888          Y88b  d88P 888        888   888        888   Y8888     888     '
        print '888   T88b  "Y88888P"  8888888P"   "Y88888P"     888           "Y8888P"  88888888 8888888 8888888888 888    Y888     888     '
        print self.ENDC
        print '\n\n'
    ##############################################################################

    ##############################################################################
    def updateCrono(self):

        dec = self.counter[3]
        inc1 = self.counter[6]
        inc2 = self.counter[7]
        if dec == 0x00:
            dec = 0xff
        else:
            dec = dec - 0x01
        if inc2 == 0xff:
            inc2 = 0x01
            inc1 = inc1 + 0x01
        else:
            inc2 = inc2 + 0x01
        self.counter[3] = dec
        self.counter[6] = inc1
        self.counter[7] = inc2
    ##############################################################################

    ##############################################################################
    def getCommand(self):
    
        if os.path.exists(self.filenameInput):
            f = open(self.filenameInput, 'r')
            command = f.readline()
            f.close()
            os.remove(self.filenameInput)
            return command
        else:
            return 'none'
    ##############################################################################

    ##############################################################################
    def sendallCharacter(self, character):

        order = self.sendallorder
        order[3] = 0x77 - character
        order[7] = character
        self.s.sendall(bytearray(order))
        data = self.s.recv(9)
        if data[8] == order[7]:
            self.sendConfirmation()
            return True
        return False
    ##############################################################################
    
    ##############################################################################
    def processCommand(self, command):

        self.printLog('The following command will be sent to the robot')
        self.printCom(command)
        n = len(command)
        #I make sure that the command ends with a 0x0d, otherwise I add it
        if command[n-1] != 0x0d:
            command = command + '\n'
        
        if command[0] == '@':
            if command.find('quit') != -1:
                self.printLog('Closing the session')
                self.exit()
        elif command[0] == '.':
            return
        else:
            return 
    ##############################################################################


