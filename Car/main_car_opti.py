from threading import Thread, Lock
import socket
import pickle
import sys
import os
import numpy as np
import time
import can
import numpy as np
from rplidar import RPLidar

######################################################################
            ######    CLASSE GLOBAL VARIABLE     ########
######################################################################
class GlobalVariables():
    def __init__(self):
        self.map_data = []
        self.dataUpdate = False
        self.mvt_cmd = "MOVstop"
        self.emergency = 0 #0 = no emergency  #1 = emergency in front #-1 = emergency in back
        self.sens_mvt = 0 #0 = stop     1 = forward     -1 = backward

    def set_data_map(self,str):
        self.map_data = str
        self.dataUpdate = True
    def write_mvt_cmd(self,str):
        self.mvt_cmd = str
    def write_emergency(self,int):
        self.emergency = int
    def write_sens_mvt(self,int):
        self.sens_mvt = int
        
        
    def read_data_map(self):
        self.dataUpdate = False
        return(self.map_data)
    def read_mvt_cmd(self):
        return(self.mvt_cmd)
    def read_emergency(self):
        return(self.emergency)
    def read_sens_mvt(self):
        return(self.sens_mvt)


######################################################################
            ######    CLASSE TCP SEND     ########
######################################################################
class MySend(Thread):
    global g_var
    def __init__(self, conn):
        Thread.__init__(self)
        self.conn = conn
        self.tab = []
        self.integer = 0
        self.backup_lidar = []
        self.backup_emergency = 0
        self.backup_IMU = []

    def run(self):
        while True:
            try:
                #SEND lidar data
                self.tab = g_var.read_data_map()
                if self.backup_lidar != self.tab:
                    self.backup_lidar = self.tab
                    tab_str = tab_str = pickle.dumps(self.tab)
                    msglen = len(tab_str)
                    msg_send = 'LID'+str(msglen)+':'
                    msg_send = msg_send.encode()
                    msg_send += tab_str
                    self.conn.sendall(msg_send)
                self.integer = g_var.read_emergency()
                if self.backup_emergency != self.integer:
                    self.backup_emergency = self.integer
                    if self.integer == -1 or self.integer == 1:
                        self.conn.sendall('URG:'.encode())
                    if self.integer == 0:
                        self.conn.sendall('FIN:'.encode())
                time.sleep(0.05)
            except(KeyboardInterrupt):
                return
            

######################################################################
            ######    CLASSE TCP RECEIVE     ########
######################################################################
class MyReceive(Thread):
    global g_var
    def __init__(self,conn):
        Thread.__init__(self)
        self.conn = conn

    def run(self):
        try:
            while True :
                dataRcv = conn.recv(16)
                g_var.write_mvt_cmd(dataRcv.decode())
        except(KeyboardInterrupt):
            return
        except:
            conn.close()


######################################################################
                ######    CLASSE US     ########
######################################################################
class MyUS(Thread):
    global g_var
    
    def __init__(self,bus):
        Thread.__init__(self)
        self.bus = bus
        self.MCM = 0x010
        self.MS = 0x100
        self.US1 = 0x000
        self.US2 = 0x001
        self.OM1 = 0x101
        self.OM2 = 0x102
    
    def run(self):
        USAG=0
        USAD=0
        USAC=0
        USDG=0
        USDD=0
        USDC=0
        obstacle_back = 0
        obstacle_front = 0
        try:
            while 1:
                msg=self.bus.recv()
                if msg.arbitration_id == self.US2:
                    # ultrason arriere gauche
                    distance = int.from_bytes(msg.data[0:2], byteorder='big')
                    USAG = distance
                    # ultrason arriere droit
                    distance = int.from_bytes(msg.data[2:4], byteorder='big')
                    USAD = distance
                    # ultrason arriere centre
                    distance = int.from_bytes(msg.data[4:6], byteorder='big')
                    USAC = distance
                if msg.arbitration_id == self.US1:
                    # ultrason av gauche
                    distance = int.from_bytes(msg.data[0:2], byteorder='big')
                    USDG = distance
                    # ultrason av droit
                    distance = int.from_bytes(msg.data[2:4], byteorder='big')
                    USDD = distance
                    # ultrason av centre
                    distance = int.from_bytes(msg.data[4:6], byteorder='big')
                    USDC = distance
                   
                #Obstacle derriere
                if USAG < 25 or USAD < 25 or USDC < 25:
                    obstacle_back = 1
                else:
                    obstacle_back = 0
                #Obstacle devant
                if USDG < 25 or USDD < 25 or USAC < 25:
                    obstacle_front = 1
                else:
                    obstacle_front = 0
                
                movement = g_var.read_sens_mvt()
                    
                #Emergency devant
                if obstacle_front == 1 and (movement == 1 or movement == 0):
                    g_var.write_emergency(1)
                    g_var.write_mvt_cmd("MOVstop")
                else:
                #Emergency derriere
                    if obstacle_back == 1 and (movement == -1 or movement == 0):
                        g_var.write_emergency(-1)
                        g_var.write_mvt_cmd("MOVstop")
                    else:
                        g_var.write_emergency(0)
        except(KeyboardInterrupt):
            return
                


######################################################################
            ######    CLASSE MOUV MANAG     ########
######################################################################

class MyMoveManag(Thread):
    global g_var
    def __init__(self, bus):
        Thread.__init__(self)
        self.bus  = can.interface.Bus(channel='can0', bustype='socketcan_native')
        
        self.MCM = 0x010
        self.MS = 0x100
        self.US1 = 0x000
        self.US2 = 0x001
        self.OM1 = 0x101
        self.OM2 = 0x102

        self.speed_cmd = 10
        self.movement = 0
        self.turn = 0
        self.enable_steering = 0
        self.enable = 0
        self.backup_order = []

    def run(self):
        
        try:
            while True:
        # Si il y a une emergency on envoie 0 sur le bus CAN pour stopper la voiture
                self.emergency = g_var.read_emergency()
                data = g_var.read_mvt_cmd()
                if (self.backup_order != data and self.emergency == 0) or (data == 'MOVforward' and self.emergency != 1) or (data == 'MOVbackward' and self.emergency != -1):
                    self.backup_order = data
                    header = data[0:3]
                    payload = data[3:]
                    
                #Traitement de la data
                    if (header == 'SPE'):  # speed
                        self.speed_cmd = int(payload)
                    elif (header == 'STE'):  # steering
                        if (payload == 'left'):
                            self.turn = -1
                            self.enable_steering = 1
                        elif (payload == 'right'):
                            self.turn = 1
                            self.enable_steering = 1
                        else:
                            self.turn = 0
                            self.enable_steering = 0
                    elif (header == 'MOV'):  # movement
                        if (payload == 'forward' and self.emergency != 1):
                            self.movement = 1
                            self.enable_speed = 1
                            g_var.write_sens_mvt(1)
                        elif (payload == 'backward' and self.emergency != -1):
                            self.movement = -1
                            self.enable_speed = 1
                            g_var.write_sens_mvt(-1)
                        else:
                            self.movement = 0
                            self.enable_speed = 0
                            g_var.write_sens_mvt(0)

                        
                    if self.enable_speed:
                        cmd_mv = (50 + self.movement*self.speed_cmd) | 0x80
                    else:
                        cmd_mv = (50 + self.movement*self.speed_cmd) & ~0x80

                    if self.enable_steering:
                        cmd_turn = 50 + self.turn*30 | 0x80
                    else:
                        cmd_turn = 50 + self.turn*30 & 0x80
                    #envoie de l'ordre au bus CAN
                    msg = can.Message(arbitration_id=self.MCM,data=[cmd_mv, cmd_mv, cmd_turn,0,0,0,0,0],extended_id=False)
                    self.bus.send(msg)
                elif self.emergency != 0:
                    self.movement = 0
                    self.enable_speed = 0
                    if self.enable_speed:
                        cmd_mv = (50 + self.movement*self.speed_cmd) | 0x80
                    else:
                        cmd_mv = (50 + self.movement*self.speed_cmd) & ~0x80

                    if self.enable_steering:
                        cmd_turn = 50 + self.turn*30 | 0x80
                    else:
                        cmd_turn = 50 + self.turn*30 & 0x80
                    #envoie de l'ordre au bus CAN
                    msg = can.Message(arbitration_id=self.MCM,data=[cmd_mv, cmd_mv, cmd_turn,0,0,0,0,0],extended_id=False)
                    self.bus.send(msg)
            
                time.sleep(0.010)
        except(KeyboardInterrupt):
            return
            
            
######################################################################
            ######    CLASS LIDAR     ########
######################################################################
class Mylidar(Thread):
    global g_var

    def __init__(self):
        Thread.__init__(self)
        PORT_NAME = '/dev/ttyUSB0'
        self.lidar = RPLidar(PORT_NAME)
        

    def run(self):
        try:
            while True:
                self.scan = []
                time.sleep(1)
                try:
                    for meas in self.lidar.iter_measurments(max_buf_meas=500):
                        self.scan.append(meas[2:])
                        if meas[0]:
                            g_var.set_data_map(self.scan)
                            meas = []
                            self.scan = []
                            time.sleep(0.001)
                except:
                    self.stop()
        except(KeyboardInterrupt):
            self.stop()
            return

    def stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()


######################################################################
            ######    MAIN LOOP     ########
######################################################################
if __name__ == "__main__":
    g_var = GlobalVariables()
    HOST = ''
    PORT = 1245
    os.system("sudo /sbin/ip link set can0 up type can bitrate 400000")

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
        exit()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()

    th_MyReceive = MyReceive(conn)
    th_MySend = MySend(conn)
    th_MyUS = MyUS(bus)
    th_MyMoveManag = MyMoveManag(bus)
    th_Mylidar = Mylidar()
    
    th_MySend.start()
    th_MyReceive.start()
    th_MyMoveManag.start()
    th_MyUS.start()
    th_Mylidar.start()
    
    try:
        while True:
            time.sleep(1)
    except(KeyboardInterrupt):
        time.sleep(10)
        th_MySend.join()
        th_MyReceive.join()
        th_MyMoveManag.join()
        th_MyUS.join()
        th_Mylidar.join()
    



