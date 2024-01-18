import board
import busio
import adafruit_rfm9x
from digitalio import DigitalInOut, Direction
from APRS import APRS
import random
import config
import time
import binascii

# configure LEDs
biasT = DigitalInOut(board.GP1)
biasT.direction = Direction.OUTPUT
biasT.value = False

TX = DigitalInOut(board.GP0)
TX.direction = Direction.OUTPUT
TX.value = False

PA = DigitalInOut(board.GP2)
PA.direction = Direction.OUTPUT
PA.value = False

# our version
VERSION = "RF.Guru_APRSLoraTest 0.1" 

print(f"{config.call} -=- {VERSION}\n")

# LoRa APRS frequency
RADIO_FREQ_MHZ = 433.775
CS = DigitalInOut(board.GP21)
RESET = DigitalInOut(board.GP20)
spi = busio.SPI(board.GP10, MOSI=board.GP11, MISO=board.GP8)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000, agc=False,crc=True)

while True:
    print(f"[{config.call}] loraRunner: Waiting for lora APRS packet ...\r", end="")
    timeout = int(config.timeout) + random.randint(1, 9)
#    biasT.value = True
#    packet = rfm9x.receive(with_header=True,timeout=timeout)
#    if packet is not None:
#        if packet[:3] == (b'<\xff\x01'):
#            try:
#                rawdata = bytes(packet[3:]).decode('utf-8')
#                print(f"\r[{config.call}] loraRunner: RSSI:{rfm9x.last_rssi} SNR:{rfm9x.last_snr} Data:{rawdata}")
#                wifi.pixel_status((100,100,0))
#                loop.create_task(tcpPost(rawdata))
#                wifi.pixel_status((0,100,0))
#            except:
#                print(f"[{config.call}] loraRunner: Lost Packet, unable to decode, skipping")
#                continue
#    time.sleep(120)
    biasT.value = False
    TX.value = True
    message = "testikkel testikkel test"
    rfm9x.send(
            bytes("{}".format("<"), "UTF-8") + binascii.unhexlify("FF") + binascii.unhexlify("01") +
            bytes("{}".format(message), "UTF-8")
        )
    time.sleep(5)
    PA.value = True
    rfm9x.send(
            bytes("{}".format("<"), "UTF-8") + binascii.unhexlify("FF") + binascii.unhexlify("01") +
            bytes("{}".format(message), "UTF-8")
        )
    PA.value = False
    time.sleep(5)
    TX.value = False

