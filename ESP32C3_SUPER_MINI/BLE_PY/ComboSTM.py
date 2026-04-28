import asyncio
import struct
from bleak import BleakScanner, BleakClient
from pynput import keyboard

DEVICE_NAME = "ESP32-C3-BLE"
CHAR_UUID = "abcd1234-5678-1234-5678-abcdef123456"

pressed_keys = set()
running = True

# ================= BUILD COMMAND =================
def build_command():
    cmd = []

    if 'w' in pressed_keys: cmd.append("F")
    if 's' in pressed_keys: cmd.append("B")
    if 'a' in pressed_keys: cmd.append("L")
    if 'd' in pressed_keys: cmd.append("R")
    if 'j' in pressed_keys: cmd.append("J")
    if 'k' in pressed_keys: cmd.append("K")
    if 'p' in pressed_keys: cmd.append("P")
    if 'h' in pressed_keys: cmd.append("H")

    if 'space' in pressed_keys:
        return "S"

    if len(cmd) == 0:
        return "S"

    return ",".join(cmd)

# ================= KEY =================
def on_press(key):
    global running

    try:
        k = key.char.lower()

        # 🔥 TEKAN Q → KELUAR
        if k == 'q':
            print("Keluar...")
            running = False
            return

        pressed_keys.add(k)

    except:
        if key == keyboard.Key.space:
            pressed_keys.add('space')

def on_release(key):
    try:
        if key.char:
            pressed_keys.discard(key.char.lower())
    except:
        if key == keyboard.Key.space:
            pressed_keys.discard('space')

# ================= RECEIVE BINARY =================
def notification_handler(sender, data):
    if len(data) == 36:
        values = struct.unpack('<9f', data)

        A0, A1, A2, A3 = values[0:4]
        P0, P1, P2, P3 = values[4:8]
        yaw = values[8]

        print(
            f"{A0:.2f} {P0:.2f}        "
            f"{A1:.2f} {P1:.2f}        "
            f"{A2:.2f} {P2:.2f}        "
            f"{A3:.2f} {P3:.2f}        "
            f"{yaw:.2f}"
        )
    else:
        print("SIZE SALAH:", len(data))

# ================= SCAN =================
async def find_device():
    print("Scanning BLE...")
    devices = await BleakScanner.discover()

    for d in devices:
        print(d.name, d.address)
        if d.name and DEVICE_NAME in d.name:
            return d.address

    return None

# ================= MAIN =================
async def main():
    global running

    address = await find_device()

    if not address:
        print("Device tidak ditemukan!")
        return

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    async with BleakClient(address) as client:
        print("Connected!")

        await client.start_notify(CHAR_UUID, notification_handler)

        print("\n=== CONTROL ===")
        print("WASD + J/K = gerak")
        print("P = STOP")
        print("H = HOME")
        print("SPACE = STOP")
        print("Q = EXIT\n")

        try:
            while running:
                cmd = build_command()
                await client.write_gatt_char(CHAR_UUID, cmd.encode())
                await asyncio.sleep(0.05)

        finally:
            # 🔥 FAILSAFE STOP
            print("Kirim STOP sebelum keluar...")
            try:
                await client.write_gatt_char(CHAR_UUID, b"S")
            except:
                pass

            await client.stop_notify(CHAR_UUID)
            print("Disconnected")

# ================= RUN =================
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nDihentikan manual (CTRL+C)")