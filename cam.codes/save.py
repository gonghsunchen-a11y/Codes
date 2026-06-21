import os

APP_ID = "rcj"
AUTO_START = "/maixapp/auto_start.txt"
APP_DIR = "/maixapp/apps/" + APP_ID
APP_MAIN = APP_DIR + "/main.py"
ROOT_MAIN = "/root/main.py"


def exists(path):
    try:
        os.stat(path)
        return True
    except:
        return


def print_first_lines(path, n=20):
    print("\n=== first lines:", path, "===")
    try:
        with open(path, "r") as f:
            for i in range(n):
                line = f.readline()
                if not line:
                    break
                print(str(i + 1) + ":", line.rstrip())
    except Exception as e:
        print("cannot read:", path, e)


print("===== RCJ Auto Start Installer =====")
print("APP_ID =", APP_ID)
print("APP_DIR =", APP_DIR)
print("APP_MAIN =", APP_MAIN)
print("ROOT_MAIN =", ROOT_MAIN)

print("\n[1] Create app folder")
try:
    os.mkdir(APP_DIR)
    print("created:", APP_DIR)
except Exception as e:
    print("mkdir:", e)

print("\n[2] Write wrapper main.py")
wrapper_code = '''from maix import time

ROOT_MAIN = "/root/main.py"

print("boot wrapper start:", ROOT_MAIN)

try:
    with open(ROOT_MAIN, "r") as f:
        code = f.read()

    exec(code, {
        "__name__": "__main__",
        "__file__": ROOT_MAIN,
    })

except Exception as e:
    print("failed to run root main:", e)

while True:
    time.sleep(1)
'''

try:
    with open(APP_MAIN, "w") as f:
        f.write(wrapper_code)
    print("wrapper written:", APP_MAIN)
except Exception as e:
    print("write wrapper failed:", e)

print("\n[3] Set auto_start.txt")
try:
    with open(AUTO_START, "w") as f:
        f.write(APP_ID)
    print("auto_start set to:", APP_ID)
except Exception as e:
    print("write auto_start failed:", e)

print("\n[4] Check files")
print("app dir exists:", exists(APP_DIR))
print("wrapper exists:", exists(APP_MAIN))
print("root main exists:", exists(ROOT_MAIN))

try:
    with open(AUTO_START, "r") as f:
        current = f.read().strip()
    print("auto_start now:", current)
except Exception as e:
    print("cannot read auto_start:", e)

print_first_lines(APP_MAIN, 35)
print_first_lines(ROOT_MAIN, 35)

print("\n===== Result =====")
if exists(APP_MAIN) and exists(ROOT_MAIN):
    print("OK: reboot/power cycle should run /root/main.py automatically.")
else:
    print("NOT OK: missing wrapper or /root/main.py.")
    