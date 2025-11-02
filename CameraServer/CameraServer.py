#   ______                                   _____                          
#  / ____/___ _____ ___  ___  _________ _   / ___/___  ______   _____  _____
# / /   / __ `/ __ `__ \/ _ \/ ___/ __ `/   \__ \/ _ \/ ___/ | / / _ \/ ___/
#/ /___/ /_/ / / / / / /  __/ /  / /_/ /   ___/ /  __/ /   | |/ /  __/ /    
#\____/\__,_/_/ /_/ /_/\___/_/   \__,_/   /____/\___/_/    |___/\___/_/     
#
import socket
import sys
import time



class CameraServer:

    ##############################################################################
    def __init__(self, robot_IP, robot_PORT):

        #Technical stuff
        self.HEADER = '\033[95m'
        self.OKBLUE = '\033[94m'
        self.OKGREEN = '\033[92m'
        self.FAIL = '\033[91m'
        self.ENDC = '\033[0m'

        #Information for the client
        self.robot_IP = robot_IP
        self.robot_PORT = robot_PORT

        #Picture name
        self.pictureName = 'picture.png'

        #Messages from the server
        self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4]
        self.serverMessage = [0x02, 0x00, 0xff, 0x44, 0x00, 0x14]
        self.countingAnswer = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0f, 0x00, 0x02]

        #Messages from the client
        self.clientHi = [0x00, 0x04, 0x04, 0x1a, 0x17, 0x02, 0xe6, 0xdf, 0x00, 0x00, 0x00, 0x00]
        self.sendSequenceOK = [0xff, 0xff, 0xff, 0xff]
        self.takePicture = [0x00, 0x04, 0x04, 0xde, 0x16, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00]
        self.sendallorder = [0xff, 0x44, 0x44, 0x6a, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00]

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #Show off
        self.showBanner()

        #Create connection
        server_address = (self.robot_IP, self.robot_PORT)
        self.s.bind(server_address)
        self.s.listen(1)
        self.connection, self.client_address = self.s.accept()
        self.printLog('Connection request from: ' + self.client_address[0])

        try:
            #Start the server action
            data = self.connection.recv(12) 
            if self.isHandshake(data):
                if not self.handleHandshake(data):
                    self.printError("The handshake was not successful")
                    self.exit()
            while True:
                data = self.connection.recv(12) 
                if self.isTakePic(data):
                    self.handlePic()
                else:
                    self.printError("Unexpected message from the client: exiting")
                    self.exit()     
        finally:
            self.printError('Connection with client closed')
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

        self.s.shutdown(socket.SHUT_RDWR)
        self.s.close()
        sys.exit()
    ##############################################################################
 
    ##############################################################################
    def showBanner(self):

        print(self.HEADER)
        print(' ______                                   _____                          ')
        print('/ ____/___ _____ ___  ___  _________ _   / ___/___  ______   _____  _____  ')
        print('/ /   / __ `/ __ `__ \\/ _ \\/ ___/ __ `/   \\__ \\/ _ \\/ ___/ | / / _ \\/ ___/ ')
        print('/ /___/ /_/ / / / / / /  __/ /  / /_/ /   ___/ /  __/ /   | |/ /  __/ /    ')
        print('\\____/\\__,_/_/ /_/ /_/\\___/_/   \\__,_/   /____/\\___/_/    |___/\\___/_/     ')
        print(self.ENDC)
        print('\n\n')
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
    def isHandshake(self, data):

        return self.isEqual(data, self.clientHi)
    ##############################################################################

    ##############################################################################
    def isTakePic(self, data):

        return self.isEqual(data, self.isTakePicture)
    ##############################################################################

    ##############################################################################
    def checkAnswer(self):

        data = self.connection.recv(4)
        return self.isEqual(data, self.sendSequenceOK)
    ##############################################################################
    
    ##############################################################################
    def handleHandshake(self, data):

        #First handshake message
        firstmessage = bytearray(self.serverHi) 
        self.connection.sendall(firstmessage)
        if not self.checkAnswer():
            self.printError("The OK message was not received")
            self.exit()

        self.printLog("Handshake was correctly done in the server") 
        return True
    ##############################################################################

    ##############################################################################
    def handlePic(self):

        #self.takeTheActualPicture()
        time.sleep(2)
        f = open(self.pictureName, 'rb')
        l = f.read(1024)
        while (l):
            self.connection.sendall(l)
            l = f.read(1024)
            f.close()
        return True
    ##############################################################################
      


