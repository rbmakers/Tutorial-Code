from machine import Pin, SPI, ADC, I2C
import utime, usys
import ustruct as struct
from nrf24l01 import NRF24L01
from lsm9ds1 import LSM9DS1

if usys.platform == "rp2": 
    spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
    cfg = {"spi": spi, "csn": 1, "ce": 0}
else:
    raise ValueError("Unsupported platform {}".format(usys.platform))

pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")
csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
spi = cfg["spi"]
nrf = NRF24L01(spi, csn, ce, payload_size=32)

nrf.open_tx_pipe(pipes[1])
nrf.open_rx_pipe(1, pipes[0])

bus = I2C(1)
lsm = LSM9DS1(bus)

while (True) :
   
    nrf.start_listening()
    utime.sleep_ms(500) 
   
    if nrf.any():
        buf = nrf.recv()
        x1Value, y1Value, x2Value, y2Value = struct.unpack("iiii", buf)
        print(x1Value, y1Value, x2Value, y2Value)
   
    nrf.stop_listening()
    utime.sleep_ms(500) 
   
    accx = int(1000*lsm.read_accel()[0])
    accy = int(1000*lsm.read_accel()[1])
    accz = int(1000*lsm.read_accel()[2])
   
    try:
        nrf.send(struct.pack("iii", accx, accy, accz))
    except OSError:
        pass