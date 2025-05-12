from machine import Pin,PWM
from MX1508 import *
import bluetooth
from ble_advertising import advertising_payload
from micropython import const
import uasyncio as asio

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)
_ADV_APPEARANCE_GENERIC_COMPUTER = const(128)

debug=0
motor_right = MX1508(25, 26) 
motor_left = MX1508(22, 23)
sp=1023
an=0
on=0
col_id=0
comand=''
pwm = PWM(Pin(33,Pin.OUT))
pwm.freq(50)
pwm.duty(0)

class BLEUART:
    def __init__(self, ble, name="MURR_Chertili", rxbuf=100):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE,))
        # Increase the size of the rx buffer and enable append mode.
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)
        self._connections = set()
        self._rx_buffer = bytearray()
        self._handler = None
        # Optionally add services=[_UART_UUID], but this is likely to make the payload too large.
        self._payload = advertising_payload(name=name, appearance=_ADV_APPEARANCE_GENERIC_COMPUTER)
        self._advertise()

    def irq(self, handler):
        self._handler = handler

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                if self._handler:
                    self._handler()

    def any(self):
        return len(self._rx_buffer)

    def read(self, sz=None):
        if not sz:
            sz = len(self._rx_buffer)
        result = self._rx_buffer[0:sz]
        self._rx_buffer = self._rx_buffer[sz:]
        return result

    def write(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._tx_handle, data)

    def close(self):
        for conn_handle in self._connections:
            self._ble.gap_disconnect(conn_handle)
        self._connections.clear()

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)
        
def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def servo(pin, angle):
    pin.duty(map(angle, 0, 180, 20, 120))
def on_rx():
    global comand,on
    on=1
    comand=uart.read().decode().strip()
    comand=comand[2:]

ble = bluetooth.BLE()
uart = BLEUART(ble)
uart.irq(handler=on_rx)    
    
async def do_it(int_ms):
    global an,on
    while 1:
        await asio.sleep_ms(int_ms)
        if debug == 1:
            print(comand)
        if comand=='507':
            #Вперёд
            motor_right.forward(sp)
            motor_left.forward(sp)
            if debug == 1:
                print("вперед")
        if comand=='606':
            #Назад
            motor_right.reverse(sp)
            motor_left.reverse(sp)
            if debug == 1:
                print("назад")
        if comand=='20:':
            #Стоп
            motor_right.stop()
            motor_left.stop()
            if debug == 1:
                print("стоп")
        if comand=='804':
            #Вправо
            motor_right.reverse(sp)
            motor_left.forward(sp)
            if debug == 1:
                print("вправо")
        if comand=='705':
            #Влево
            motor_right.forward(sp)
            motor_left.reverse(sp)
            if debug == 1:
                print("влево")
        if comand=='10;' and on:
            if debug == 1:
                print("камера вверх")
            an+=30
            on=0
            if an>180:
                an=180
        if comand=='309' and on:
            if debug == 1:
                print("камера вниз")
            an-=30
            on=0
            if an<0:
                an=0
        servo(pwm, an)

# define loop
loop = asio.get_event_loop()

#create looped tasks
loop.create_task(do_it(10))
# loop run forever
loop.run_forever()

#uart.close()
