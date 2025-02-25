/*ECE 590: ENGINEERING PROJECT II
 * MULTI-PURPOSE QUADCOPTER
 * : Gas Quality/Leakage Monitoring
 * EC/09/14
 * KITHINJI MURIUNGI
 * 2018/19
 * Supervisor: Mr. S.B.Kifalu
*/

//MQ2 GAS SENSOR

import network
import urequests
import machine
import time

# WiFi Credentials
WIFI_SSID = "Panthers"
WIFI_PASSWORD = "qwerty90"

# Firebase Credentials
FIREBASE_HOST = "https://home-automation-7fc48.firebaseio.com"
FIREBASE_AUTH = "KKDcaIEcTWOHGvHHdtlemDkf1Q37alqqpUcwbSt2"
FIREBASE_URL = f"{FIREBASE_HOST}/air_condition.json?auth={FIREBASE_AUTH}"

# MQ2 Gas Sensor Pin (Assumes ADC pin for ESP32)
mq2GasSensor = machine.ADC(0)  # ADC0 on ESP32

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    print("Connecting to WiFi", end="")
    while not wlan.isconnected():
        print(".", end="")
        time.sleep(0.5)
    print("\nConnected to WiFi! IP:", wlan.ifconfig()[0])


def send_to_firebase(value):
    try:
        response = urequests.put(FIREBASE_URL, json=value)
        print("Data sent to Firebase:", response.text)
        response.close()
    except Exception as e:
        print("Error sending data to Firebase:", e)


def main():
    connect_wifi()
    while True:
        gas_reading = mq2GasSensor.read()  # Read sensor value
        print("Gas Sensor Value:", gas_reading)
        send_to_firebase(gas_reading)
        time.sleep(0.2)  # 200ms delay


if __name__ == "__main__":
    main()
