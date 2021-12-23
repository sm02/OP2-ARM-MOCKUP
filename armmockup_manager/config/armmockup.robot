[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE   | DEFAULT JOINT
/dev/ttyUSB0 | 57600      | l_sho_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME      | BULK READ ITEMS

dynamixel | /dev/ttyUSB0 | 1   | MX-28          | 1.0      | l_sho_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 2   | MX-28          | 1.0      | l_sho_roll    | present_position
dynamixel | /dev/ttyUSB0 | 3   | MX-28          | 1.0      | l_el          | present_position

