import asyncio
from bleak import BleakScanner, BleakClient
import keyboard

DEVICE_NAME = "ESP32-C3-BLE"
CHAR_UUID = "abcd1234-5678-1234-5678-abcdef123456"

# ================= SCAN DEVICE =================
async def find_device():
    print("Scanning BLE...")
    devices = await BleakScanner.discover()

    for d in devices:
        if d.name == DEVICE_NAME:
            print("Found:", d.address)
            return d.address

    return None

# ================= MAIN =================
async def main():
    address = await find_device()

    if address is None:
        print("Device tidak ditemukan!")
        return

    async with BleakClient(address) as client:
        print("Connected ke ESP32!")

        print("\n=== CONTROL ===")
        print("W = MAJU")
        print("S = MUNDUR")
        print("A = KIRI")
        print("D = KANAN")
        print("SPACE = STOP")
        print("Q = EXIT\n")

        while True:
            if keyboard.is_pressed('w'):
                await client.write_gatt_char(CHAR_UUID, b'F')
                print("MAJU")
                await asyncio.sleep(0.2)

            elif keyboard.is_pressed('s'):
                await client.write_gatt_char(CHAR_UUID, b'B')
                print("MUNDUR")
                await asyncio.sleep(0.2)

            elif keyboard.is_pressed('a'):
                await client.write_gatt_char(CHAR_UUID, b'L')
                print("KIRI")
                await asyncio.sleep(0.2)

            elif keyboard.is_pressed('d'):
                await client.write_gatt_char(CHAR_UUID, b'R')
                print("KANAN")
                await asyncio.sleep(0.2)

            elif keyboard.is_pressed('space'):
                await client.write_gatt_char(CHAR_UUID, b'S')
                print("STOP")
                await asyncio.sleep(0.2)

            elif keyboard.is_pressed('q'):
                print("Keluar...")
                break

            await asyncio.sleep(0.01)

asyncio.run(main())