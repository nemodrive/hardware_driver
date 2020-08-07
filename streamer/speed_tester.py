CAN_LOG = "/home/alex/work/AI-MAS/projects/AutoDrive/dev/car_data_collection/data/session_18_nov_original/can_raw_short.log"

import can
import cantools
import time

CAN_CMD_NAMES = dict({
            # cmd_name, data_name, can_id
            "speed": ("SPEED_SENSOR", "SPEED_KPS", "354"),
            "steer": ("STEERING_SENSORS", "STEER_ANGLE", "0C6"),
            "brake": ("BRAKE_SENSOR", "PRESIUNE_C_P")
        })

db = cantools.db.load_file('logan.dbc')
can_bus = can.interface.Bus('vcan0', bustype='socketcan')

with open(CAN_LOG) as fp:
    while True:
        can_entry = fp.readline()

        if can_entry:
            can_entry_data = can_entry.strip().split(" ")[2]
            arb_id = can_entry_data.split("#")[0].strip()
            msg_data = can_entry_data.split("#")[1].strip()

            print("arb_id=", arb_id, len(arb_id))
            print("msg_data", msg_data)

            message = can.Message(arbitration_id=int(arb_id, 16),
                                  data=bytearray.fromhex(msg_data))

            can_bus.send(message)

            #input("Press Enter to continue sending messages ...")
            time.sleep(0.01)
        else:
            break



# message = can_bus.recv()
# db.decode_message(message.arbitration_id, message.data)