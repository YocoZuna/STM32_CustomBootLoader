import serial
from intelhex import IntelHex
import binascii
from Crypto.Cipher import AES
import struct
import argparse
import time
import threading
import sys
parser = argparse.ArgumentParser(
    prog="STM32_UART_FLASHER",
    description= "This program loads .hex files to STM32 that uses custom bootloader.\n"
    "Before using this script make sure that you flash STM32 with proper bootloader",

    epilog="Made by: Dawid Zadlo Enjoy :)\n\r"

)

parser.add_argument('-f','--hex',help="Path to hex file",type=str)           # positional argument
parser.add_argument('-p', '--port',help="serial port to which uC is connected\n"
                    "On Linux most likely: /dev/ttyACMx\n"
                    "On Windows COMXX",type=str)      # option that takes a value
parser.add_argument('-b', '--baudrate',help="Baudrate of UART port\n default 115200",default=115200,type=int) 
parser.add_argument('-m', '--massErase',default=True,help="Should perform mass erase of uC? default TRUE!")  
parser.add_argument('-u', '--uC',help="Name of uC that you want to flash")  
parser.add_argument('-v', '--verbose',default=False) 
class TimeOutError(Exception):
    def __init__(self, *args):
        super().__init__(*args)
        print("TimeOutError")
        sys.exit()
class FlashWriteError(Exception):
    def __init__(self, *args):
        super().__init__(*args)
        print("FlashWriteError")
        sys.exit()
class FlashEraseError(Exception):
    def __init__(self, *args):
        super().__init__(*args)
        print("FlashEraseError")
        sys.exit()
class ACKnowlageError(Exception):
    def __init__(self, *args):
        super().__init__(*args)
        print("ACKnowlageError")
        sys.exit()
class ProgramError(Exception):
    def __init__(self, *args):
        super().__init__(*args)
        print("ProgramError")
        sys.exit()

class CRC:
    
    def __init__(self,poly,initialCRC):
        self.poly = poly
        self.initalCRC = initialCRC


    def Calculate(self,inputData):

        _crc = self.initalCRC
        for d in inputData:
            _crc ^= d
            for i in range(32):
                if _crc & 0x80000000:
                    _crc = (_crc<<1)^self.poly

                else:
                    _crc=_crc<<1

        return (_crc & 0xFFFFFFFF)


class Loader():

    AES_KEY = "40:23:3C:4D:36:39:57:74:79:43:78:53:51:39:12:19"

    ACK       = 0x79
    NACK      = 0x1F
    CMD_ERASE = 0x43
    CMD_GETID = 0x02
    CMD_WRITE = 0x2b
    CMD_PROGRAMMING_DONE = 0x53

    STM32_TYPE = {
        '0x410': "STM32F103RB",
        '0x415': "STM32L152RG",
        '0x417': "STM32L053R8",
        '0x421': "STM32F446RE",
        '0x431': "STM32F411RE",
        '0x433': "STM32F401RE",
        '0x437': "STM32L152RE",
        '0x439': "STM32F302R8",
        '0x438': "STM32F334R8",
        '0x440': "STM32F030R8",
        '0x442': "STM32F091RC",
        '0x446': "STM32F303RE",
        '0x447': "STM32L073RZ",
        '0x448': "STM32F070RB/STM32F072RB",
        '0x458': "STM32F410RB",
    }

    def __init__(self,COMport,baudrate):
        self._uC = serial.Serial(COMport,baudrate,timeout=3)
        self.crc = CRC(0x04C11DB7,0xFFFFFFFF)

    def _convertTouint32(self,data):
        return list(struct.pack("I",data))
    def _createMessage(self,*args):

        message = list()

        for i in args:
            #Calculate crc from data
            if (type(i)== list )or (type(i)==tuple) or (type(i) == set):

                for y in i:
                    message.append(y)

            else:
                message.append(i)

        message.append(self.crc.Calculate(message))
        buffor = []
        for i in message:
            if i>255:
                buffor.extend(self._convertTouint32(i))
            else:
                buffor.append(i)
        return buffor

    def  _encrypt(self,data):
        final = []
        _aes_key = binascii.unhexlify(Loader.AES_KEY.replace(':',""))
        _aes_encryptor = AES.new(_aes_key,AES.MODE_ECB)

        encryptedData = _aes_encryptor.encrypt(bytes(data))
        dataBytes = struct.unpack("IIII", encryptedData)

        return dataBytes
    def eraseFlash(self,whole=False,**kwargs)->bool:
        """
        *
        *
        *
        * A write command has the following structure:
        *
        * --------------------------------------------
        * | CMD_ID | number of sectors     |  CRC32  |
        * | 1 byte |     to erase          | 4 bytes |
        * |--------|-----------------------|---------|
        * |  0x43  |   N r OFF for all     |   CRC   |
        * --------------------------------------------
        *
        *
        """
        if whole == True:
            sectorsToErase = 0xFF
        else:
            sectorsToErase = hex(kwargs.get("sectors"))

        self._uC.flush()
        self._uC.write(self._createMessage(Loader.CMD_ERASE,sectorsToErase))
        ret  = self._uC.read(1)
        if len(ret) == 1:
            if struct.unpack("b",ret)[0] != Loader.ACK:
                raise ACKnowlageError()
            else:
                return True
        else:
            raise TimeOutError()

    def getID(self) -> str:

        """
        * A GET_ID command has the following structure:
        *
        * --------------------
        * | CMD_ID |  CRC32  |
        * | 1 byte | 4 bytes |
        * |--------|---------|
        * |  0x02  |   CRC   |
        * --------------------
        *
        """
        self._uC.flush()
        self._uC.write(self._createMessage(Loader.CMD_GETID))
        ret  = self._uC.read(1)
        if len(ret) == 1:

            if struct.unpack("b",ret)[0] != Loader.ACK:
                raise ACKnowlageError()
            else:
                ret = self._uC.read(2)
                if len(ret)==2:
                    return Loader.STM32_TYPE[(hex(struct.unpack("h",ret)[0]))]

                else:
                    raise ProgramError("GetID command failed")

        else:
            raise TimeOutError()

    def programFlash(self,hexFilePath):

        """
        * A write command has the following structure:
        *
        * ----------------------------------------
        * | CMD_ID | starting address  |  CRC32  |
        * | 1 byte |     4 byte        | 4 bytes |
        * |--------|-------------------|---------|
        * |  0x2b  |    0x08004000     |   CRC   |
        * ----------------------------------------
        *
        * The second message has the following structure
        *
        * ------------------------------
        * |    data bytes    |  CRC32  |
        * |      16 bytes    | 4 bytes |
        * |------------------|---------|
        * | BBBBBBBBBBBBBBBB |   CRC   |
        * ------------------------------
        """

        hexFile = IntelHex()
        hexFile.loadhex(hexFilePath)
        endAddress,startAddress = hexFile.maxaddr(),hexFile.minaddr()
        currentAddres = startAddress
        hexFileContent = hexFile.todict()

        while currentAddres <= endAddress:
            startAddress = currentAddres
            data = []
            # Loop over to get 16 bytes
            for i in range(16):
                try:
                    data.append(hexFileContent[currentAddres])
                except KeyError:
                    data.append(0xFF)
                currentAddres+=1

            self._uC.flush()

            self._uC.write(self._createMessage(Loader.CMD_WRITE,startAddress))

            ret  = self._uC.read(1)
            if len(ret) == 1:

                if struct.unpack("b",ret)[0] != Loader.ACK:
                    raise ACKnowlageError()
                else:
                    self._uC.write(self._createMessage(self._encrypt(data)))

                    ret  = self._uC.read(1)
                    if len(ret) == 1:
                        if struct.unpack("b",ret)[0] == Loader.ACK:
                            yield{"Start":hex(startAddress),"End":hex(startAddress+16)}
                    else:
                        raise FlashWriteError("Error during writing to flash memory")

            else:
                raise TimeOutError()

    def programingDone(self) -> str:

        """
        * Inform that programing is done
        *
        * --------------------
        * | CMD_ID |  CRC32  |
        * | 1 byte | 4 bytes |
        * |--------|---------|
        * |  0x69  |   CRC   |
        * --------------------
        *
        """
        self._uC.flush()
        self._uC.write(self._createMessage(Loader.CMD_PROGRAMMING_DONE))

def eraseFlash():   
    STM32.eraseFlash(whole=args.massErase)

def dotPrinter(threadToFolow:threading.Thread,interval):
    while(threadToFolow.is_alive()):
        print(".",end='')
        sys.stdout.flush()
        time.sleep(interval)


def program(hexFile,verbose):
    print("\nStart of programming",end="")
    while(1):
        for e in  STM32.programFlash(hexFile):
            if verbose ==True:
                sys.stdout.write(f"Programming addresses: {e['Start']}-{e['End']}")
                sys.stdout.flush()
        STM32.programingDone()
        break  

args = parser.parse_args()
if (args.hex==None) or (args.port==None) or (args.baudrate==None)or (args.massErase==None) or(args.uC==None):
    print("Param missing check once more\n")
    parser.print_help()

else:
    
    print("""
************************
    SELECTED SETTINGS  
                        
    hex file: %s       
    COM port: %s       
    baudrate: %d       
    mass erasse: %d     
                            
************************
    """%(args.hex,args.port,args.baudrate,args.massErase))
    

    STM32 = Loader(args.port,args.baudrate)
    if args.uC not in STM32.STM32_TYPE.values():
        print("Selected uC will not be detected properly because it is not supported")
    else:
        found = STM32.getID()
        print(f"\nFound uC:{found}")
        if args.uC != found:
            print("Wrong uC detected..Abort")
            sys.exit()
    print('Erasing flash',end="")
    eraseFlashThread = threading.Thread(target=eraseFlash)
    dotPrinterThread = threading.Thread(target=dotPrinter,args=[eraseFlashThread,0.5])
    
    eraseFlashThread.start()
    dotPrinterThread.start()
    eraseFlashThread.join()
    dotPrinterThread.join()
        
    programerThread = threading.Thread(target=program,args=[args.hex,args.verbose])
    dotPrinterThread2 = threading.Thread(target=dotPrinter,args=[programerThread,0.5])
    programerThread.start()
    if args.verbose ==False:
        dotPrinterThread2.start()
    programerThread.join()
    if args.verbose ==False:
        dotPrinterThread2.join()
    print("\nDone")









