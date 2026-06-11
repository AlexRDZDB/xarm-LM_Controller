import os
from dotenv import load_dotenv
from xarm.wrapper import XArmAPI

load_dotenv()

arm = XArmAPI(os.getenv('ROBOT_IP'))
arm.motion_enable(enable=True)
arm.clean_error()
arm.clean_warn()
arm.set_mode(0)
arm.set_state(0)

print(f'State: {arm.get_state()}')
print(f'Error: {arm.get_err_warn_code()}')
print(f'Angles: {arm.get_servo_angle()}')

arm.disconnect()