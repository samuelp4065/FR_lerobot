from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode
import time

name = "shoulder_pan"
motors = {name: Motor(id=3, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100)}

bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
bus.connect()

# Enable torque if not already done
bus.write("Torque_Enable", name, 1, normalize=False)

# Read current position
pos = bus.read("Present_Position", name, normalize=False)
print("Current =", pos)
print(name, "model =", bus._id_to_model_dict[3])

# Quarter-turn step (ticks)
# step = 0

# # Command new position = current + 1/4 revolution
# target = min(pos + step, 4095)
# bus.write("Goal_Position", name, target, normalize=False)
# print("Target =", target)

# time.sleep(1.0)

bus.disconnect()



#모터마다 값다르게 움직이기
# from lerobot.motors.feetech import FeetechMotorsBus
# from lerobot.motors.motors_bus import Motor, MotorNormMode
# import time

# # Define all motors (friendly name -> Motor object)
# motors = {
#     "shoulder_pan":   Motor(id=1, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
#     "shoulder_lift":  Motor(id=2, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
#     "elbow":          Motor(id=3, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
#     "wrist_pitch":    Motor(id=4, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
#     "wrist_roll":     Motor(id=5, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
#     "gripper":        Motor(id=6, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
# }

# bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
# bus.connect()

# # Enable torque on all motors
# for name in motors.keys():
#     bus.write("Torque_Enable", name, 1, normalize=False)

# # Read positions
# positions = {name: bus.read("Present_Position", name, normalize=False)
#              for name in motors.keys()}
# print("Current positions:", positions)

# # Quarter-turn step (e.g., 1024 ticks ~ 90°)
# step = 5

# # Command each motor to move +step (clamped to 0…4095)
# for name, pos in positions.items():
#     target = min(pos + step, 4095)
#     bus.write("Goal_Position", name, target, normalize=False)
#     print(f"{name} target -> {target}")

# time.sleep(1.0)

# bus.disconnect()






# from lerobot.motors.feetech import FeetechMotorsBus
# from lerobot.motors.motors_bus import Motor, MotorNormMode
# import time

# name = "shoulder_pan"
# motors = {name: Motor(id=1, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100)}

# bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
# bus.connect()

# # Essentials (once per power-up)
# bus.write("Torque_Enable",        name, 1,    normalize=False)  # torque ON
# bus.write("Operating_Mode",       name, 3,    normalize=False)  # position mode
# bus.write("Min_Position_Limit",   name, 0,    normalize=False)  # open limits
# bus.write("Max_Position_Limit",   name, 4095, normalize=False)
# bus.write("Max_Torque_Limit",     name, 1023, normalize=False)  # full torque
# bus.write("Minimum_Startup_Force",name, 50,   normalize=False)  # ensure it starts
# # Optional profile (skip if your servo ignores these)
# try:
#     bus.write("Profile_Velocity",     name, 300, normalize=False)
#     bus.write("Profile_Acceleration", name, 50,  normalize=False)
# except Exception:
#     pass

# # Move: go to mid-range, then nudge
# pos  = bus.read("Present_Position", name, normalize=False)
# mid  = 2047
# bus.write("Goal_Position", name, mid,          normalize=False)
# time.sleep(1.2)
# bus.write("Goal_Position", name, min(mid+400, 4095), normalize=False)
# time.sleep(1.0)
# bus.write("Goal_Position", name, max(mid-400, 0),    normalize=False)
# time.sleep(1.0)

# bus.disconnect()




# # diag_move_feetech.py
# from lerobot.motors.feetech import FeetechMotorsBus
# from lerobot.motors.motors_bus import Motor, MotorNormMode
# import time

# MOTOR_NAME = "shoulder_pan"
# MOTOR_ID   = 1
# MODEL_KEY  = "sts3215"   # <-- IMPORTANT: use a key that exists in your codebase (NOT "sts3215" if it's missing)

# motors = {
#     MOTOR_NAME: Motor(id=MOTOR_ID, model=MODEL_KEY, norm_mode=MotorNormMode.RANGE_0_100)
# }

# bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
# print("Connecting …")
# bus.connect()

# # ---- 0) Confirm we have the right ID on the wire (quick scan if needed) ----
# def scan_ids():
#     found = []
#     for test_id in range(1, 16):  # scan first 15 IDs; expand if needed
#         try:
#             # Try reading a universal register; fallback to 'ID'
#             val = bus._read_by_id("Present_Position", test_id, normalize=False)
#             found.append(test_id)
#         except Exception:
#             pass
#     return found

# try:
#     # Try talking to the configured ID
#     _ = bus.read("Present_Position", MOTOR_NAME, normalize=False)
#     print(f"✅ Servo at configured ID={MOTOR_ID} responded.")
# except Exception:
#     ids = scan_ids()
#     if not ids:
#         raise SystemExit("❌ No servos responded. Check port (/dev/ttyACM0), wiring, power, and baud.")
#     else:
#         print(f"⚠️ ID {MOTOR_ID} didn’t respond. Found active IDs: {ids}. "
#               f"Update MOTOR_ID or reprogram the servo’s ID.")

# # ---- 1) Show the exact register names available for your model ----
# try:
#     # Model used internally for this ID:
#     model_used = bus._id_to_model_dict[MOTOR_ID]
#     ctrl = bus.model_ctrl_table[model_used]
#     print(f"Model key in use: {model_used}")
#     print("Available registers (first 30):", list(ctrl.keys())[:30])
# except Exception as e:
#     print("Could not list control table keys:", e)

# # ---- 2) Torque ON (try all common keys; only one will succeed) ----
# for key in ("Torque_Enable", "Enable_Torque", "TorqueEnable", "Torque On"):
#     try:
#         bus.write(key, MOTOR_NAME, 1, normalize=False)
#         print(f"Torque enabled via '{key}'")
#         break
#     except Exception:
#         pass
# else:
#     print("⚠️ Could not enable torque (key name mismatch). If it still moves later, that’s fine.")

# # ---- 3) Put in position (joint) mode if your model exposes it ----
# for key, val in (("Operating_Mode", 3), ("OperatingMode", 3), ("Mode", 0), ("Mode", 1)):
#     try:
#         bus.write(key, MOTOR_NAME, val, normalize=False)
#         print(f"Set {key}={val} (position/joint)")
#         break
#     except Exception:
#         pass

# # ---- 4) Open angle limits so moves aren’t blocked ----
# for cw_key, ccw_key in (("Min_Angle_Limit", "Max_Angle_Limit"),
#                         ("CW_Angle_Limit", "CCW_Angle_Limit"),
#                         ("Angle_Limit_CW", "Angle_Limit_CCW")):
#     try:
#         bus.write(cw_key,  MOTOR_NAME, 0,     normalize=False)
#         bus.write(ccw_key, MOTOR_NAME, 4095,  normalize=False)
#         print(f"Angle limits set via '{cw_key}/{ccw_key}' to [0,4095]")
#         break
#     except Exception:
#         pass

# # ---- 5) Set modest profile speed/accel if supported ----
# for vel_key in ("Profile_Velocity", "Moving_Speed"):
#     try:
#         bus.write(vel_key, MOTOR_NAME, 300, normalize=False)
#         print(f"Velocity set via '{vel_key}'")
#         break
#     except Exception:
#         pass
# for acc_key in ("Profile_Acceleration",):
#     try:
#         bus.write(acc_key, MOTOR_NAME, 50, normalize=False)
#         print(f"Acceleration set via '{acc_key}'")
#         break
#     except Exception:
#         pass

# # ---- 6) Read pos and command a big move; verify motion ----
# pos0 = bus.read("Present_Position", MOTOR_NAME, normalize=False)
# print("Present_Position =", pos0)

# targets = [pos0 + 800, pos0 - 800]  # big enough to see movement
# for i, tgt in enumerate(targets, 1):
#     try:
#         bus.write("Goal_Position", MOTOR_NAME, int(tgt), normalize=False)
#         print(f"[{i}/2] Goal_Position <- {int(tgt)}")
#         time.sleep(1.5)
#         pos1 = bus.read("Present_Position", MOTOR_NAME, normalize=False)
#         moved = abs(pos1 - pos0)
#         print(f"New Present_Position = {pos1} (Δ={moved})")
#         if moved > 50:
#             print("✅ Movement detected.")
#         else:
#             print("⚠️ No movement yet. Check mode/limits/torque/model key.")
#         pos0 = pos1
#     except Exception as e:
#         print("Write/read error:", e)

# # ---- 7) Bonus: check status bits if your table has them ----
# for key in ("Moving", "Is_Moving", "Error", "Hardware_Error_Status", "Voltage", "Present_Temperature"):
#     try:
#         print(key, "=", bus.read(key, MOTOR_NAME, normalize=False))
#     except Exception:
#         pass

# bus.disconnect()
# print("Done.")
