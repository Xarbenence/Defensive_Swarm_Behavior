from microbit import *
import time

commandHeaderAndAddress = "55AA11"
OBJECT_TRACKING = "0100"
OBJECT_CLASSIFICATION = "0600"
checkOnceAgain = False

time.sleep(2)

i = 0
lastCmdSent = 0

class Arrow:
    def __init__(self, xTail, yTail , xHead , yHead, ID):
        self.xTail=xTail
        self.yTail=yTail
        self.xHead=xHead
        self.yHead=yHead
        self.ID=ID
        self.learned= True if ID > 0 else False
        self.type="ARROW"

class Block:
    def __init__(self, x, y , width , height, ID):
        self.x = x
        self.y=y
        self.width=width
        self.height=height
        self.ID=ID
        self.learned= True if ID > 0 else False
        self.type="BLOCK"


def unhexlify(s): # code from kind gent on the internet
    return bytes(int(s[i:i+2], 16) for i in range(0, len(s), 2))

def calculateChecksum(hexStr):
    total = 0
    for i in range(0, len(hexStr), 2):
        total += int(hexStr[i:i+2], 16)
    hexStr = hex(total)[-2:]
    return hexStr


def cmdToBytes(cmd):
        #return bytes.fromhex(cmd)  *need to use micropython equivalent*
        bytestring = unhexlify(cmd)
        return bytestring # unhexlify Converts binary data to hexadecimal representation. Returns bytes string.


def writeToHuskyLens(cmd):
    global lastCmdSent
    lastCmdSent = cmd
    reg_addr = 12 # this is the register we need to write to
    byte = reg_addr.to_bytes(1,'big')
    data = bytearray(byte+cmd) # list o' bytes, starting with memory address
    i2c.write(0x32,data) # write cmd to register 12 of the the HL (address 0x32)


def splitCommandToParts(cmd):
    # print(f"We got this str=> {str}")
    headers = cmd[0:4]
    address = cmd[4:6]
    data_length = int(cmd[6:8], 16)
    command = cmd[8:10]
    if(data_length > 0):
        data = cmd[10:10+data_length*2]
    else:
        data = []
    checkSum = cmd[2*(6+data_length-1):2*(6+data_length-1)+2]

    return [headers, address, data_length, command, data, checkSum]


def getBlockOrArrowCommand():
    byteString = b''
    for i in range(5):
        byteString += i2c.read(0x32,1) #bytes(([i2c.read(0x32,1)]))  # removed cast to bytes on the i2c read
    for i in range(byteString[3]+1): # removed cast to int on byteString[3]
        byteString += i2c.read(0x32,1) #bytes(([i2c.read(0x32,1)])) # removed cast to bytes on the i2c read

    hexstring = "".join("{:02x}".format(x) for x in byteString) # does the same as byteString.hex(), but we don't have ubinascii on the micro:bit
    commandSplit = splitCommandToParts(hexstring)
    isBlock = True if commandSplit[3] == "2a" else False
    return (commandSplit[4],isBlock)


def processReturnData(numIdLearnFlag=False, frameFlag=False):
    global checkOnceAgain
    inProduction = True
    byteString=""
    if(inProduction):
        #try:
            byteString = b''
            for i in range(5):
                #byte = i2c.read(0x32,1) # read one byte
                #print(byte)
                byteString += i2c.read(0x32,1) #bytes(([i2c.read(0x32,1)])) # read one byte from the HL at a time, removed cast to bytes on the read
                #print(byteString)
            for i in range(byteString[3]+1): # removed the cast to int in the second for loop (unnecessary)
                byteString += i2c.read(0x32,1) #bytes(([i2c.read(0x32,1)])) # removed cast to bytes
                #print(byteString)

            hexstring = "".join("{:02x}".format(x) for x in byteString) # does the same as byteString.hex(), but we don't have ubinascii on the micro:bit
            commandSplit = splitCommandToParts(hexstring)

            # print(commandSplit)
            if(commandSplit[3] == "2e"):
                checkOnceAgain=True
                return "Knock Recieved"
            else:
                returnData = []
                numberOfBlocksOrArrow = int(
                    commandSplit[4][2:4]+commandSplit[4][0:2], 16)
                numberOfIDLearned = int(
                    commandSplit[4][6:8]+commandSplit[4][4:6], 16)
                frameNumber = int(
                    commandSplit[4][10:12]+commandSplit[4][8:10], 16)
                isBlock=True
                for i in range(numberOfBlocksOrArrow):
                    tmpObj= getBlockOrArrowCommand()
                    isBlock=tmpObj[1]
                    returnData.append(tmpObj[0])

                # isBlock = True if commandSplit[3] == "2A"else False

                finalData = []
                tmp = []
                # print(returnData)
                for i in returnData:
                    tmp = []
                    for q in range(0, len(i), 4):
                        low=int(i[q:q+2], 16)
                        high=int(i[q+2:q+4], 16)
                        if(high>0):
                            val=low+255+high
                        else:
                            val=low
                        tmp.append(val)
                    finalData.append(tmp)
                    tmp = []
                checkOnceAgain=True
                ret=convert_to_class_object(finalData,isBlock)
                if(numIdLearnFlag):
                    ret.append(numberOfIDLearned)
                if(frameFlag):
                    ret.append(frameNumber)
                return ret

        #except:
            #if(checkOnceAgain):
             #   timeout=5
             #   checkOnceAgain=False
             #   timeout=.5
            #    return processReturnData()
            #print("Read response error, please try again")
           # return []

def knock():
    cmd = cmdToBytes(commandHeaderAndAddress+"002c3c")
    writeToHuskyLens(cmd)
    return processReturnData()

def count():
    cmd = cmdToBytes(commandHeaderAndAddress+"002030")
    writeToHuskyLens(cmd)
    return len(processReturnData())

def convert_to_class_object(data,isBlock):
    tmp=[]
    for i in data:
        if(isBlock):
            obj = Block(i[0],i[1],i[2],i[3],i[4])
        else:
            obj = Arrow(i[0],i[1],i[2],i[3],i[4])
        tmp.append(obj)
    return tmp


def Obj_Class(): # change the HL to object classification mode
    cmd = commandHeaderAndAddress + "022d" + OBJECT_CLASSIFICATION
    cmd += calculateChecksum(cmd)
    cmd = cmdToBytes(cmd) # cmd is completed here
    writeToHuskyLens(cmd)

def Obj_Track():
    cmd = commandHeaderAndAddress + "022d" + OBJECT_TRACKING
    cmd += calculateChecksum(cmd)
    cmd = cmdToBytes(cmd) # cmd is completed here
    writeToHuskyLens(cmd)

def blocks():
    cmd = cmdToBytes(commandHeaderAndAddress+"002131")
    writeToHuskyLens(cmd)
    return processReturnData()

#start = 0
#i = 0
#while True:

    #if start == 0:
      #  guests = i2c.scan()
      #  print(guests)
    #if len(guests) > 1 and start == 0:
        #Obj_Class()
       # start = 1
    #if start == 1:
       # print(count())
       # if count() > 0:
           # try:
               # object = blocks()[0]
               # ID = object.ID
            #x = object.x
            #y = object.y
            #height = object.height
            #data = [x,y,height]
           #     print("ID",ID)
           # except:
           #     continue

        #except:
           # ret = processReturnData()
           # print(ret[1])

