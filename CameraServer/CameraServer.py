#   ______                                   _____                          
#  / ____/___ _____ ___  ___  _________ _   / ___/___  ______   _____  _____
# / /   / __ `/ __ `__ \/ _ \/ ___/ __ `/   \__ \/ _ \/ ___/ | / / _ \/ ___/
#/ /___/ /_/ / / / / / /  __/ /  / /_/ /   ___/ /  __/ /   | |/ /  __/ /    
#\____/\__,_/_/ /_/ /_/\___/_/   \__,_/   /____/\___/_/    |___/\___/_/     
#
import socket
import sys
import time



class RobotServer:

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

        #Messages from the server
        #self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4, 0x02, 0x8c, 0x02, 0xa5, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00]
        self.serverHi = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0a, 0x00, 0xa4]
        self.serverMessage = [0x02, 0x00, 0xff, 0x44, 0x00, 0x14]
        self.countingAnswer = [0x00, 0x00, 0x00, 0x04, 0x00, 0x0f, 0x00, 0x02]

        #Messages from the client
        self.clientHi = [0x00, 0x04, 0x04, 0x1a, 0x17, 0x02, 0xe6, 0xdf, 0x00, 0x00, 0x00, 0x00]
        self.sendSequenceOK = [0xff, 0xff, 0xff, 0xff]
        self.startCounting = [0x00, 0x04, 0x04, 0xde, 0x16, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00]
        self.sendallorder = [0xff, 0x44, 0x44, 0x6a, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00]

        #Show off
        self.showBanner()

        #Create connection
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
                if self.isCounter(data):
                    self.handleCounter(data)
                else:
                    self.printError("Unexpected message from the client: exiting")
                    self.exit()     
        finally:
            self.printError('Connection with client closed')
            self.exit()
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
        print '   ______                                   _____                          '
        print '/ ____/___ _____ ___  ___  _________ _   / ___/___  ______   _____  _____  '
        print '/ /   / __ `/ __ `__ \/ _ \/ ___/ __ `/   \__ \/ _ \/ ___/ | / / _ \/ ___/ ' 
        print '/ /___/ /_/ / / / / / /  __/ /  / /_/ /   ___/ /  __/ /   | |/ /  __/ /    '
        print '\____/\__,_/_/ /_/ /_/\___/_/   \__,_/   /____/\___/_/    |___/\___/_/     '
        print self.ENDC
        print '\n\n'
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
    def isCounter(self, data):

        thedata = bytearray(data)
        counter = bytearray(self.startCounting)
 
        if counter[0] == thedata[0] and counter[1] == thedata[1] and counter[2] == thedata[2] and counter[4] == thedata[4] and counter[5] == thedata[5]:
            return True
        return False 
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
        message = bytearray('Hello stupid robot\x0d\x0a\x0d\x0a', 'ascii')
        firstmessage.extend(message)
        self.connection.sendall(firstmessage)
        if not self.checkAnswer():
            self.printError("The OK message was not received")
            self.exit()

        #Second handshake message
        message2 = "This is some random information"
        n = len(message2)
        message = bytearray(message2, 'ascii')
        server = list(self.serverMessage)
        server.append(0x00)
        server.append(n)
        secondmessage = bytearray(server)
        secondmessage.extend(message)
        self.connection.sendall(secondmessage)
        if not self.checkAnswer():
            self.printError("The OK message was not received")
            self.exit()

        #Third handshake message
        message3 = "This is the ROBOT information"
        n = len(message3)
        message = bytearray(message3, 'ascii')
        server = list(self.serverMessage)
        server.append(0x00)
        server.append(n)    
        thirdmessage = bytearray(server)
        thirdmessage.extend(message)
        self.connection.sendall(thirdmessage)
        if not self.checkAnswer():
            self.printError("The OK message was not received")
            self.exit()

        self.printLog("Handshake was correctly done in the server") 
        return True
    ##############################################################################

    ##############################################################################
    def handleCounter(self, data):

        thedata = bytearray(data)
        answer = list(self.countingAnswer)
        answer.append(ord(data[6]))
        answer.append(ord(data[7]))
        message = bytearray(answer)
        time.sleep(1)
        self.connection.sendall(message)
        if not self.checkAnswer():
            self.printError("The OK message was not received")
            self.exit()
        self.printLog("Counter message with id " + str(ord(data[6])) + ' ' + str(ord(data[7])) + ' received by server')
        return True
    ##############################################################################
      


