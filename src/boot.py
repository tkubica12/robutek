import webrepl
from time import sleep_ms

yourWifiSSID = "TomNet"
yourWifiPassword = "Lopata12doma"
def connect():
    import network
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(yourWifiSSID, yourWifiPassword)
        for retry in range(10):
            if sta_if.isconnected():
                break
            sleep_ms(500)
    print('network config:', sta_if.ifconfig())
    webrepl.start()

def showip():
    import network
    sta_if = network.WLAN(network.STA_IF)
    print('network config:', sta_if.ifconfig())

# the connect() line can be uncommented if you want the ESP to connect
# to WiFi automatically when it boots. If you uncomment this, be sure
# you have tested it first with your network.
#
# connect()
# 