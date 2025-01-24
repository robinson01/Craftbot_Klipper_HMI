import threading
import queue
import requests
import serial
import time

# Moonraker IP address
moonraker_ip = "http://192.168.0.138"

# Initialize UART connection
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Replace with your serial port
    baudrate=230400,      # Set baud rate
    bytesize=serial.EIGHTBITS,  # Data bits (8)
    parity=serial.PARITY_NONE,  # Parity (None)
    stopbits=serial.STOPBITS_ONE,  # Stop bits (1)
    xonxoff=False,        # Software flow control (False)
    rtscts=False,         # Hardware (RTS/CTS) flow control (False)
    dsrdtr=False,         # Hardware (DSR/DTR) flow control (False)
    timeout=1             # Read timeout (1 second)
)
##############################################################
# Serial commands
##############################################################
scr_init_req = "M1003S69"  
scr_init_response = [
    "ok \r\n",
    "ok  T:0 t:0 B:0 b:0 f:0 U:260 Z:0 E:0 F:0 k:0 W:0 r:0 d:0 s:100 e:100 i:0 c:0 $:0\r\n",
    "ok v:1.1.12701/4 - 2023-07-17\r\n",
    "ok  T:101 B:101\r\n",
    "ok u:51203020-3931374d-31303820-17303030\r\n",
    "ok  $:15677\r\n",
    "r:1069635 d:37816096\r\n",
    "ok z:47977,3768,-1000,1,0\r\n",
    "ok\r\n",
    "r:14403 d:18941728\r\n",
    "ok z:0,0,-1000,1,0\r\n"
]
scr_upd = "ok  $:15677"

##############################################################
#Moonraker variables
##############################################################

in_remote_print_job = False

status = "/printer/info"

endpoints_machine_status = [
    "/printer/objects/query?idle_timeout=state,printing_time",
    "/printer/objects/query?extruder=target,temperature",
    "/printer/objects/query?heater_bed=target,temperature",
    "/printer/objects/query?fan=speed",
    "/printer/objects/query?toolhead=homed_axes,position,print_time,estimated_print_time"
    "/printer/objects/query?idle_timeout=state,printing_time"
 ]
 
endpoints_printing_status = [
    "/printer/objects/query?gcode_move=speed,speed_factor,extrude_factor",
    "/printer/objects/query?display=progress"
 ]

##############################################################
# Queue for passing data between threads
##############################################################
data_queue = queue.Queue()
shutdown_flag = threading.Event()

##############################################################
#Moonraker helper functions
##############################################################
def query_moonraker(endpoint):
    try:
        response = requests.get(moonraker_ip + endpoint)
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        print(f"Error querying Moonraker: {e}")
        return None

def send_gcode(gcode):
    try:
        response = requests.post(f"{moonraker_ip}/printer/gcode/script", json={"script": gcode})
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        print(f"Error sending G-code: {e}")
        return None

##############################################################
#Serial helper functions
##############################################################
def ser_write_upd(msg):
    ser.write(msg.encode())
    ser.write(scr_upd.encode())
    print(msg + scr_upd)


##############################################################
#Thread for serial communication
##############################################################
def serial_thread():
    try:
        while not shutdown_flag.is_set():
            if ser.is_open and ser.in_waiting > 0:
                uart_data = ser.readline().decode().strip()
                data_queue.put(uart_data)  # Put received data in the queue
                if uart_data == scr_init_req:
                    for message in scr_init_response:
                        ser_write_upd(message)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        ser.close()
        
##############################################################
#Thread for Moonraker communication
##############################################################
def moonraker_thread():
    extruder_temp, extruder_tgt, heater_bed_temp, heater_bed_tgt, home_int, th_pos  = 0, 0, 0, 0, 0, [0,0,0,0] 
    th_pos_float = [0.0, 0.0, 0.0, 0.0]
    printer_state = ""
    while not shutdown_flag.is_set():
        for endpoint in endpoints_machine_status:
            response = query_moonraker(endpoint)
            if response:
                if 'extruder' in response['result']['status']:
                    extruder_temp = response['result']['status']['extruder']['temperature']
                    extruder_tgt = response['result']['status']['extruder']['target']
                elif 'heater_bed' in response['result']['status']:
                    heater_bed_temp = response['result']['status']['heater_bed']['temperature']
                    heater_bed_tgt = response['result']['status']['heater_bed']['target']
                elif 'toolhead' in response['result']['status']:
                    home_string = response['result']['status']['toolhead']['homed_axes']
                    th_pos = response['result']['status']['toolhead']['position']
                    for axis in range(3):
                        th_pos[axis] = int(1000 * th_pos[axis])
                    home_int = 0
                    for axis in 'zyx':
                        home_int <<= 1
                        if axis in home_string:
                            home_int += 1
                elif 'idle_timeout' in response['result']['status']:
                    printer_state =  response['result']['status']['idle_timeout']['state']
                else:
                    print(endpoint + " data not available")
                    
                    
            else:
                print("Failed to retrieve machine information.")
        data_queue.put((extruder_temp, extruder_tgt, heater_bed_temp, heater_bed_tgt, home_int, th_pos, printer_state))
        #time.sleep(1)

##############################################################
#Main function
##############################################################
def main():
    printer_status = query_moonraker(status)
    print(f"printer status: {printer_status}\r\n")
    
    threading.Thread(target=serial_thread, daemon=True).start()
    threading.Thread(target=moonraker_thread, daemon=True).start()

    try:
        while not shutdown_flag.is_set():
            while not data_queue.empty():
                data = data_queue.get()
                if isinstance(data, tuple):
                    extruder_temp, extruder_tgt, heater_bed_temp, heater_bed_tgt, home_int, th_pos, printer_state = data
                    if printer_state == "printing" and in_remote_print_job == False:
                        ser_write_upd("ok n:1\r\n")
                        in_remote_print_job == True
                    elif printer_state != "printing" and in_remote_print_job == True:
                        ser_write_upd("ok n:2\r\n")
                        in_remote_print_job == False
                else:
                    uart_data = data
                    if uart_data.startswith("G") or uart_data.startswith("M104") or \
                       uart_data.startswith("M140") or uart_data.startswith("M109") or \
                       uart_data.startswith("M190") or uart_data.startswith("M84"):
                        response = send_gcode(uart_data)
                        if response:
                            ser_write_upd(f"ACK:{response}\n")
                    elif uart_data.startswith("M1104"):
                        ser_write_upd(f"ok T:{extruder_temp} t:{extruder_tgt} B:{heater_bed_temp} b:{heater_bed_tgt} R:{home_int} Z:{th_pos[2]} \r\n")

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting...\r\n")
        shutdown_flag.set()

if __name__ == "__main__":
    main()