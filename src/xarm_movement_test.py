import os
import time
from dotenv import load_dotenv
from xarm.wrapper import XArmAPI

load_dotenv()

arm = XArmAPI(os.getenv('ROBOT_IP'))
arm.motion_enable(enable=True)
arm.clean_error()
arm.clean_warn()
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], speed=20, wait=True)
print('At home')
time.sleep(1)

arm.set_servo_angle(angle=[0, -30, 0, 30, 0, 30], speed=20, wait=True)
print('At target')
time.sleep(1)

arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], speed=20, wait=True)
print('Back home')

arm.disconnect()