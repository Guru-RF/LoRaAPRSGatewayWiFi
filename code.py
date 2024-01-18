import time
import rtc
import board
import busio
import adafruit_rfm9x
from digitalio import DigitalInOut, Direction
from adafruit_esp32spi import adafruit_esp32spi, adafruit_esp32spi_wifimanager
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from adafruit_esp32spi import PWMOut
import adafruit_rgbled
import adafruit_requests as requests
import asyncio
from adafruit_datetime import datetime
from APRS import APRS
import random
import config
from microcontroller import watchdog as w
from watchdog import WatchDogMode
import microcontroller
from secrets import secrets

# our version
VERSION = "RF.Guru_APRSiGate 0.1" 

def _format_datetime(datetime):
  return "{:02}/{:02}/{} {:02}:{:02}:{:02}".format(
    datetime.tm_mon,
    datetime.tm_mday,
    datetime.tm_year,
    datetime.tm_hour,
    datetime.tm_min,
    datetime.tm_sec,
  )

def purple(data):
  stamp = "{}".format(_format_datetime(time.localtime()))
  return "\x1b[38;5;104m[" + str(stamp) + "] " + data + "\x1b[0m"

def green(data):
  stamp = "{}".format(_format_datetime(time.localtime()))
  return "\r\x1b[38;5;112m[" + str(stamp) + "] " + data + "\x1b[0m"

def blue(data):
  stamp = "{}".format(_format_datetime(time.localtime()))
  return "\x1b[38;5;14m[" + str(stamp) + "] " + data + "\x1b[0m"

def yellow(data):
  return "\x1b[38;5;220m" + data + "\x1b[0m"

def red(data):
  return "\x1b[1;5;31m -- " + data + "\x1b[0m"

# configure biasT
biasT = DigitalInOut(board.GP0)
biasT.direction = Direction.OUTPUT
biasT.value = False

if config.biasT is True:
    biasT.value = True

# wait for console
time.sleep(2)

print(red(f"{config.call} -=- {VERSION}\n"))

try:
    from secrets import secrets
except ImportError:
    print(red("WiFi secrets are kept in secrets.py, please add them there!"))
    raise

esp32_cs = DigitalInOut(board.GP17)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP13)

# Clock MOSI(TX) MISO(RX)
spi = busio.SPI(board.GP18, board.GP19, board.GP16)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
      print(yellow("ESP32 found and in idle mode"))
print(yellow("Firmware version: " + (esp.firmware_version).decode("utf-8")))
print(yellow("MAC addr: " + str([hex(i) for i in esp.MAC_address])))

RED_LED = PWMOut.PWMOut(esp, 25)
GREEN_LED = PWMOut.PWMOut(esp, 26)
BLUE_LED = PWMOut.PWMOut(esp, 27)
status_light = adafruit_rgbled.RGBLED(RED_LED, GREEN_LED, BLUE_LED)
wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets, status_light)

## Connect to WiFi
print(yellow("Connecting to WiFi..."))
wifi.connect()
print(yellow("Connected!"))
  
print(yellow("Connected to: [" + str(esp.ssid, "utf-8") + "]\tRSSI:" + str(esp.rssi)))
print()

# Initialize a requests object with a socket and esp32spi interface
socket.set_interface(esp)
requests.set_socket(socket, esp)

# configure watchdog
w.timeout=5
w.mode = WatchDogMode.RESET
w.feed()

now = None
while now is None:
    w.feed()
    try:
        now = time.localtime(esp.get_time()[0])
    except OSError:
        pass
rtc.RTC().datetime = now

# SEND iGate Postition
wifi.pixel_status((100,100,0))
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(10)
try:
    socketaddr = socket.getaddrinfo(config.aprs_host, config.aprs_port)[0][4]
    s.connect(socketaddr)
    aprs = APRS()
    pos = aprs.makePosition(config.latitude, config.longitude, -1, -1, config.symbol)
    altitude = "/A={:06d}".format(int(config.altitude*3.2808399))
    comment = config.comment + altitude
    ts = aprs.makeTimestamp('z',now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)
    message = f'user {config.call} pass {config.passcode} vers {VERSION}\n'
    s.send(bytes(message, 'utf-8'))
    message = f'{config.call}>APRFGI,TCPIP*:@{ts}{pos}{comment}\n'
    s.send(bytes(message, 'utf-8'))
    print(purple(f"[{config.call}] iGatePossition: {message}"), end="")
    wifi.pixel_status((0,100,0))
except:
    print(purple(f"[{config.call}] Connect to ARPS {config.aprs_host} {config.aprs_port} Failed ! Lost Packet ! Restarting System !"))
    microcontroller.reset()


async def iGateAnnounce():
    global s
    while True:
        w.feed()
        temp = microcontroller.cpus[0].temperature
        freq = microcontroller.cpus[1].frequency/1000000
        rawpacket = f'{config.call}>APRFGI,TCPIP*:>Running on RP2040 t:{temp}C f:{freq}Mhz\n'
        try:
            s.send(bytes(rawpacket, 'utf-8'))
        except:
            print(purple(f"[{config.call}] iGateStatus: Reconnecting to ARPS {config.aprs_host} {config.aprs_port}"))
            s.close()
            socket.set_interface(esp)
            requests.set_socket(socket, esp)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            try:
                socketaddr = socket.getaddrinfo(config.aprs_host, config.aprs_port)[0][4]
                s.connect(socketaddr)
                rawauthpacket = f'user {config.call} pass {config.passcode} vers {VERSION}\n'
                s.send(bytes(rawauthpacket, 'utf-8'))
                s.send(bytes(rawpacket, 'utf-8'))
            except:
                print(purple(f"[{config.call}] Connect to ARPS {config.aprs_host} {config.aprs_port} Failed ! Lost Packet ! Restarting system !"))
                microcontroller.reset()
        print(purple(f"[{config.call}] iGateStatus: {rawpacket}"),end="")
        aprs = APRS()
        pos = aprs.makePosition(config.latitude, config.longitude, -1, -1, config.symbol)
        altitude = "/A={:06d}".format(int(config.altitude*3.2808399))
        comment = config.comment + altitude
        ts = aprs.makeTimestamp('z',now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)
        message = f'{config.call}>APRFGI,TCPIP*:@{ts}{pos}{comment}\n'
        try:
            s.send(bytes(message, 'utf-8'))
        except:
            print(purple(f"[{config.call}] iGateStatus: Reconnecting to ARPS {config.aprs_host} {config.aprs_port}"))
            s.close()
            socket.set_interface(esp)
            requests.set_socket(socket, esp)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            try:
                socketaddr = socket.getaddrinfo(config.aprs_host, config.aprs_port)[0][4]
                s.connect(socketaddr)
                rawauthpacket = f'user {config.call} pass {config.passcode} vers {VERSION}\n'
                s.send(bytes(rawauthpacket, 'utf-8'))
                s.send(bytes(message, 'utf-8'))
            except:
                print(purple(f"[{config.call}] iGateStatus: Connect to ARPS {config.aprs_host} {config.aprs_port} Failed ! Lost Packet ! Restarting system !"))
                microcontroller.reset()
        
        print(purple(f"[{config.call}] iGatePossition: {message}"),end="")
        await asyncio.sleep(15*60)


async def tcpPost(packet):
    global s
    rawpacket = f'{packet}\n'
    try:
        s.send(bytes(rawpacket, 'utf-8'))
    except:
        print(purple(f"[{config.call}] AprsTCPSend: Reconnecting to ARPS {config.aprs_host} {config.aprs_port}"))
        s.close()
        socket.set_interface(esp)
        requests.set_socket(socket, esp)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        try:
            socketaddr = socket.getaddrinfo(config.aprs_host, config.aprs_port)[0][4]
            s.connect(socketaddr)
            rawauthpacket = f'user {config.call} pass {config.passcode} vers {VERSION}\n'
            s.send(bytes(rawauthpacket, 'utf-8'))
            s.send(bytes(rawpacket, 'utf-8'))
        except:
            print(purple(f"[{config.call}] AprsTCPSend: Reconnecting to ARPS {config.aprs_host} {config.aprs_port} Failed ! Lost Packet ! Restarting system !"))
            microcontroller.reset()
    print(blue(f"[{config.call}] AprsTCPSend: {packet}"))
    await asyncio.sleep(0)


async def loraRunner(loop,w):
    # LoRa APRS frequency
    RADIO_FREQ_MHZ = 433.775
    CS = DigitalInOut(board.GP21)
    RESET = DigitalInOut(board.GP20)
    spi = busio.SPI(board.GP10, MOSI=board.GP11, MISO=board.GP8)
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000, agc=False,crc=True)

    while True:
        await asyncio.sleep(0)
        timeout = int(config.timeout) + random.randint(1, 9)
        print(purple(f"[{config.call}] loraRunner: Waiting for lora APRS packet timeout:{timeout} ...\r"),end="")
        packet = rfm9x.receive(w,with_header=True,timeout=timeout)
        if packet is not None:
            if packet[:3] == (b'<\xff\x01'):
                try:
                    rawdata = bytes(packet[3:]).decode('utf-8')
                    print(green(f"[{config.call}] loraRunner: RSSI:{rfm9x.last_rssi} SNR:{rfm9x.last_snr} Data:{rawdata}"))
                    wifi.pixel_status((100,100,0))
                    loop.create_task(tcpPost(rawdata))
                    wifi.pixel_status((0,100,0))
                except:
                    print(purple(f"[{config.call}] loraRunner: Lost Packet, unable to decode, skipping"))
                    continue


async def main():
   loop = asyncio.get_event_loop()
   loraR = asyncio.create_task(loraRunner(loop,w))
   loraA = asyncio.create_task(iGateAnnounce())
   await asyncio.gather(loraR, loraA)


asyncio.run(main())