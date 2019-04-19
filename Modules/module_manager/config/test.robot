[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyUSB0 | 1000000   | head_pan

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL   | PROTOCOL | DEV NAME    | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 19  | MX-28   | 1.0      | head_pan    | present_position, position_p_gain, position_i_gain, position_d_gain, present_torque
dynamixel | /dev/ttyUSB0 | 20  | MX-28   | 1.0      | head_tilt   | present_position, position_p_gain, position_i_gain, position_d_gain, present_torque
#sensor    | /dev/ttyUSB0 | 200 | CM-740  | 1.0      | cm-740      | button, present_voltage
