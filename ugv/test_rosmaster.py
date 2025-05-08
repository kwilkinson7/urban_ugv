# test_rosmaster_clean.py

import time
from ugv.rosmaster import Rosmaster

def main():
    bot = Rosmaster(com="/dev/myserial", debug=True)
    bot.create_receive_threading()
    time.sleep(0.1)

    # print("Enabling auto-reporting...")
    bot.set_auto_report_state(True)

    # print("Beeping...")
    bot.set_beep(100)

    # print("Setting motor forward...")
    bot.set_motor(50, 50, 50, 50)
    time.sleep(3)

    # print("Stopping motor...")
    bot.set_motor(0, 0, 0, 0)

    # print("Reading battery voltage...")
    # print("Battery:", bot.get_battery_voltage(), "V")

    # print("Getting PID settings...")
    # print("PID:", bot.get_motion_pid())

    # print("Reading accelerometer...")
    # print("Accel:", bot.get_accelerometer_data())

    # print("Done.")

if __name__ == "__main__":
    main()
