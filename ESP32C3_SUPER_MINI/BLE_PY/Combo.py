import asyncio
from bleak import BleakScanner, BleakClient
from pynput import keyboard

DEVICE_NAME = "ESP32-C3-BLE"
CHAR_UUID = "abcd1234-5678-1234-5678-abcdef123456"

pressed_keys = set()

# ================= BUILD COMMAND =================
def build_command():
    cmd = []

    if 'w' in pressed_keys: cmd.append("F")
    if 's' in pressed_keys: cmd.append("B")
    if 'a' in pressed_keys: cmd.append("L")
    if 'd' in pressed_keys: cmd.append("R")
    if 'j' in pressed_keys: cmd.append("J")
    if 'k' in pressed_keys: cmd.append("K")

    if 'space' in pressed_keys:
        return "S"

    if len(cmd) == 0:
        return "S"

    return ",".join(cmd)

# ================= KEY EVENTS =================
def on_press(key):
    try:
        pressed_keys.add(key.char.lower())
    except:
        if key == keyboard.Key.space:
            pressed_keys.add('space')

def on_release(key):
    try:
        pressed_keys.discard(key.char.lower())
    except:
        if key == keyboard.Key.space:
            pressed_keys.discard('space')

# ================= BLE =================
async def find_device():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            return d.address
    return None

async def main():
    address = await find_device()
    if not address:
        print("Device tidak ditemukan!")
        return

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    async with BleakClient(address) as client:
        print("Connected!")

        while True:
            cmd = build_command()

            # 🔥 KIRIM TERUS TANPA CEK PERUBAHAN
            await client.write_gatt_char(CHAR_UUID, cmd.encode())

            print("KIRIM:", cmd)

            await asyncio.sleep(0.05)  # 20Hz

asyncio.run(main())