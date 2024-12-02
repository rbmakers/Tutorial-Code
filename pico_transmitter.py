from machine import Pin, SPI, ADC
import utime, usys
import ustruct as struct
from nrf24l01 import NRF24L01

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

nrf.open_tx_pipe(pipes[0])
nrf.open_rx_pipe(1, pipes[1])

x1Axis = ADC(Pin(27))
y1Axis = ADC(Pin(26))
x2Axis = ADC(Pin(28))
y2Axis = ADC(Pin(29))

ACCX = 0
ACCY = 0
ACCZ = 0

while (True):
    
    nrf.stop_listening()
    utime.sleep_ms(150)
    
    x1Value = x1Axis.read_u16()
    y1Value = y1Axis.read_u16()
    x2Value = x2Axis.read_u16()
    y2Value = y2Axis.read_u16()
    
    try:
        nrf.send(struct.pack("iiii", x1Value, y1Value, x2Value, y2Value))
    except OSError:
        pass
    
    nrf.start_listening()
    utime.sleep_ms(100)
        
    if nrf.any():
        buf = nrf.recv()
        ACCX, ACCY, ACCZ = struct.unpack("iii", buf)
        print(ACCX, ACCY, ACCZ)