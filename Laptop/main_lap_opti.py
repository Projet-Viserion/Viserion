import socket
import pickle
import time
import os
import sys
import math
import wx
import wx.xrc
import wx.adv
import struct
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from matplotlib import style
from threading import Thread

''' Class Obstacle used to store the classified obstacles '''

class Obstacle:
    #Cette classe a pour but definir ce qui est un obstacle observé par le LiDAR.
    #En gros,
    def __init__(self, points, color):
        # points should be an numpy array
        self.vel = 0
        self.acc = 0
        self.color = color
        self.trajectory = [] # La trajectoire de chaque objet sera définit par comment les points bouge.
        self.centroid = [] # Le centroid sera la moyenne de la position de chaque points
        self.setPoints(points) # Chaque objet sera réconnu par un ensemble de points
        
    def setPoints(self, points):
        self.points = points
        self.centroid = [np.mean(points[:,0]), np.mean(points[:,1])]
        self.trajectory.append(self.centroid)
        lenTraj = len(self.trajectory)
        if lenTraj > 3:
            self.updateVel(lenTraj)
            self.updateAcc(lenTraj)
    def getPoints(self):
        return self.points
    def getColor(self):
        return self.color
    def updateVel(self, lenTraj):
        self.vel = (self.trajectory[lenTraj-1] - self.trajectory[lenTraj-2])/(0.15*1000)
    def updateAcc(self, lenTraj):
        self.acc = ((self.trajectory[lenTraj-1] - self.trajectory[lenTraj-2])/(0.15*1000) - (self.trajectory[lenTraj-2] - self.trajectory[lenTraj-3])/(0.15*1000))/(0.15)
    
    
''' Class Frame store the data of the current Lidar acquisition '''

class Frame:
    def __init__(self,dataMapXY):
        self.dataMapXY = dataMapXY
        self.obstacles = []
        self.colors = ['gray', 'green', 'black', 'yellow', 'fuchsia', 'blueviolet', 'orange', 'lightsalmon', 'darkturquoise']    
        self.identifyObstacles()

    def identifyObstacles(self):
        # il faut faire une gestion des objets:
        # Si deux objets possède deux centroid differents, ils ne peuvent pas se fondre dans un seule.
        points = [[self.dataMapXY[0,0], self.dataMapXY[0,1]]]
        meas_p = np.array([0,0])
        cont = 0
        sphereRadius = 100
        # brand new identification
        for (i,meas) in enumerate(self.dataMapXY):
            if i > 0:
                if  self.insideNeighborhood('sphere', [meas[0],meas[1]], [meas_p[0], meas_p[1]], sphereRadius):
                    points.append([meas[0],meas[1]])
                else:
                    if len(points) > 1:
                        i = cont%(len(self.colors))
                        self.obstacles.append(Obstacle(np.array(points), 'gray'))
                        cont += 1
                        points = [[meas[0], meas[1]]]
                    else :
                        points = [[meas[0], meas[1]]]
            else:
                pass
            meas_p[0] = meas[0]
            meas_p[1] = meas[1]
        '''
        if self.insideNeighborhood('sphere', [meas_p[0], meas_p[1]], [self.dataMapXY[0,0], self.dataMapXY[0,1]], sphereRadius):
            startObst = self.obstacles.pop(0).getPoints()
            lastObst = self.obstacles.pop(len(obstacles)-1).getPoints()
            lastObst = np.concatenate(startObst, lastObst)
            self.obstacles.append(Obstacle(lastObst,'red')) 
        '''
        # new identification:
        # This one must use 
        if points != []:
            self.obstacles.append(Obstacle(np.array(points), self.colors[0]))
            cont += 1
            points = [[meas[0], meas[1]]]

    def insideNeighborhood(self,normType,point1, point2, normValue):
        if normType == 'sphere':
            if distance(point1,point2) < normValue:
                return True
            else:
                return False
        elif normType == 'circSector':
            return False
            pass
        elif normType == 'max':
            return False
            pass
        else:
            return False
            pass
    def showObstacles(self,ax1):
        #ax1.plot(self.dataMapXY[:,0], self.dataMapXY[:,1], '.',color = 'fuchsia')
        
        for obs in self.obstacles:
            points = obs.getPoints()
            #ax1.plot(points[:,0],points[:,1], '.',color = obs.getColor(),markersize=10)
            ax1.plot(obs.centroid[0], obs.centroid[1], 'o', color = 'black')

            # max distance between centroid and point of obstacle
            max_dist = 0
            for (i, p) in enumerate(points):
                dist = math.sqrt((p[1]-obs.centroid[1])**2 + (p[0]-obs.centroid[0])**2)
                if dist > max_dist :
                    max_dist = dist
                    if max_dist >= 1000:
                        print(points)
                        print(self.dataMapXY)

            # plot circle around the mobile obstacle
            angle = np.linspace(0, 2*np.pi, 100)
            radius = max_dist + 10
            x1 = radius*np.cos(angle)+obs.centroid[0]
            x2 = radius*np.sin(angle)+obs.centroid[1]

            ax1.plot(x1, x2, 'g--')

            
def distance(P,P0):
    return math.sqrt((P[1]-P0[1])**2 + (P[0]-P0[0])**2)
#def identifyObstacles(points):



'''Share variable for the laptop program. We use a class to make it easier to manage'''
class global_variable():
    def __init__(self):
        self.data_map = []
        self.mvt_cmd = "MOVstop"
        self.emergency = False
        self.data_map_1 = []

            
    def write_data_map_1(self,str):
        self.data_map_1 = str
    def write_data_map(self,str):
        self.data_map = str
    def write_mvt_cmd(self,str):
        self.mvt_cmd = str
    def write_emergency(self,bool):
        self.emergency = bool
        
    def read_data_map_1(self):
        return(self.data_map_1)
    def read_data_map(self):
        return(self.data_map)
    def read_mvt_cmd(self):
        return(self.mvt_cmd)
    def read_emergency(self):
        return(self.emergency)

class receive(Thread):
    global g_var, s
    def __init__(self):
        Thread.__init__(self)
        self.backup = []
    def run(self):
        try:
            while True :
                msg_recv = s.recv(1)
                h = str(msg_recv)
                j = h.find(':')
                while j == -1:
                    msg_recv += s.recv(1)
                    h = str(msg_recv)
                    j = h.find(':')
                header = msg_recv[:j-1].decode()
                if header[:3] == 'LID':
                    msglen = int(header[3:j-2])
                    fullmsg = msg_recv[j-1:]
                    while len(fullmsg) != msglen:
                        msg_recv = s.recv(msglen-len(fullmsg))
                        fullmsg += msg_recv
                    try:
                        fullmsg = pickle.loads(bytes(fullmsg))
                        self.backup = fullmsg
                    except:
                        fullmsg = self.backup
                    g_var.write_data_map(fullmsg)
                else:
                    if header[:3] == 'URG':
                        g_var.write_mvt_cmd("MOVstop")
                        g_var.write_emergency(True)
                    else: 
                        if header[:3] == 'FIN':
                            g_var.write_emergency(False)

        except(KeyboardInterrupt):
            s.close()
            return


class send(Thread):
    global g_var, s
    def __init__(self):
        Thread.__init__(self)
        self.backup = []
    def run(self):
        try:
            while True:
                msg_send = g_var.read_mvt_cmd()
                if msg_send != self.backup:
                    self.backup = msg_send
                    msg_send = msg_send.encode()
                    s.send(msg_send)
                else:
                    time.sleep(0.1)
                    
        except(KeyboardInterrupt):
            return
        except:
            return
            
'''Function for dynamic mapping'''
def dynamic_map(i):
    global g_var
    global first_value
    global nb_frame
    global all_frame
    global size_of_frame
    global previous_map
    global t1
    # init des listes dans lesquelles les points seront stockées
    x_to_print = []
    y_to_print = []
    data_map = g_var.read_data_map()
    if len(data_map) > 0:
        data_map = np.asarray(data_map)
        if g_var.read_mvt_cmd() == "MOVstop" :
        
            # If it is the first value of the acquistion, it means that it is the
                # first acquistion and  so no previous map exists.
                # So we put "same_map" to "False"
            if first_value == 1 :
                same_map = False
            else :
                same_map = np.array_equal(data_map,previous_map)
            
            # We check if the map in "data_map" is tha same than the previous one.
                # It it is the case, we didn't treat the data
            if same_map == False :

                # We mesure the time taken to update "data_map"

                # We translate "data_map" into a usable array "current_frame with :
                    # r = distance to the point from the Lidar
                    # theta_degree = angle of the lidar for the point
                for (i, meas) in enumerate(data_map):
                    if meas[1] != 0:
                        r = float(meas[1])
                        theta_degree = int(meas[0])
                        if first_value == 1:
                            first_value = 0
                            current_frame = np.array([[theta_degree,r]])
                        else:
                            current_frame = np.append(current_frame,[[theta_degree,r]],axis=0)
                        y_to_print.append(r*math.cos(theta_degree))
                        x_to_print.append(r*math.sin(theta_degree))

                # Number of frames used for the drifting mean
                max_frame = 71

                # Store information in different arrays :
                    # nb_frame = number of frames used for the moment
                    # all_frame = store all the measure of all the frames used for
                        # the drifting mean
                    # size_of_frame = store the size of each frames used
                if first_value == 0 :
                    # Append frames
                    if nb_frame == 1:
                        all_frame = current_frame
                        size_of_frame = np.array([len(current_frame)])
                        nb_frame = nb_frame + 1
                    elif nb_frame == max_frame :
                        all_frame = np.delete(all_frame, slice(0, size_of_frame[0]), axis=0)
                        size_of_frame = np.delete(size_of_frame,0)

                        all_frame = np.append(all_frame,current_frame,axis=0)
                        size_of_frame = np.append(size_of_frame,[len(current_frame)],axis=0)

                    else:
                        all_frame = np.append(all_frame,current_frame,axis=0)
                        size_of_frame = np.append(size_of_frame,[len(current_frame)],axis=0)
                        nb_frame = nb_frame + 1

                    x_mean = []
                    y_mean = []
                    x_move = []
                    y_move = []
                    x_immo = []
                    y_immo = []

                    # distance max between an immobile point and the mean
                    moving_limit = 200

                    # Drifting mean "x_mean" and "y_mean" calculation and
                        # classification of the point of the current_frame into
                        # 2 arrays :
                        # "x_move" and "y_move" for the mobile points
                        # "x_immo" and "y_immo" for the immobile points
                    for thetaa in range(360):
                        ind = np.where(all_frame[:,0] == thetaa)
                        if len(ind[0]) != 0 and len(ind[0]) > (nb_frame/2):
                            mean_r = np.mean(all_frame[ind[0],1])
                            y_mean.append(mean_r*math.cos(math.radians(thetaa)))
                            x_mean.append(mean_r*math.sin(math.radians(thetaa)))

                            ind_current = np.where(current_frame[:,0] == thetaa)
                            current_r = np.mean(current_frame[ind_current[0],1])

                            if abs(mean_r - current_r) > moving_limit :
                                x_move.append(current_r*math.sin(math.radians(thetaa)))
                                y_move.append(current_r*math.cos(math.radians(thetaa)))
                            else:
                                x_immo.append(current_r*math.sin(math.radians(thetaa)))
                                y_immo.append(current_r*math.cos(math.radians(thetaa)))

                    first_value = 1

                    x = np.array([x_to_print])
                    x = x.transpose()
                    y = np.array([y_to_print])
                    y = y.transpose()
                    x_move = np.array([x_move])
                    x_move = x_move.transpose()
                    y_move = np.array([y_move])
                    y_move = y_move.transpose()
                    dataMapXY = np.concatenate((x_move,y_move),axis = 1)

                    ax1.clear()

                    ax1.plot(x_immo,y_immo,'r.')
                    ax1.plot(x_move,y_move,'g.')
                    #ax1.plot(x_mean,y_mean,'r.')
                    ax1.plot(0,0, 'D',color = 'blue')
                    ax1.plot([-250,250],[0,0], 'b')
                    ax1.plot([-250,250],[-850,-850], 'b')
                    ax1.plot([-250,-250],[0,-850], 'b')
                    ax1.plot([250,250],[0,-850], 'b')

                    if len(x_move) != 0:
                        mapping = Frame(dataMapXY)
                        mapping.showObstacles(ax1)

                    ax1.set_xlim([-2000,2000])
                    ax1.set_ylim([-2000,2000])
                    #ax1.axis('equal')
                    ax1.grid()

        else :
            nb_frame = 1
            first_value = 1
            
            x = []
            y = []
            
            same_map = np.array_equal(data_map,previous_map)
            
            # passage des coordonnées polaires en cartésiennes
            for (i, meas) in enumerate(data_map):
                r = float(meas[1])
                theta = math.radians(float(meas[0]))
                y.append(r*math.cos(theta))
                x.append(r*math.sin(theta))

            ax1.clear()
            ax1.plot(x, y,'r.')
            #ax1.plot(x_mov, y_mov,'gD')
            ax1.plot(0,0, 'D',color = 'blue')
            ax1.plot([-250,250],[0,0],'b-')
            ax1.plot([-250,250],[-850,-850],'b-')
            ax1.plot([-250,-250],[0,-850],'b-')
            ax1.plot([250,250],[0,-850],'b-')
            
            ax1.set_xlim([-2000,2000])
            ax1.set_ylim([-2000,2000])
            #ax1.axis('equal')
            ax1.grid()
            
            
        
        previous_map = data_map
    
    
'''GUI Function'''
class MyFrame ( wx.Frame ):
    global g_var
    def __init__( self, parent ):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = "Car control", pos = wx.Point(0,0), size = wx.Size( 750,750 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

        self.gofoward = wx.Button( self, wx.ID_ANY, "Forward or Arrow up",wx.Point(275,125), wx.Size( 200,50 ), 0 )
        self.gofoward.SetFont( wx.Font( 13, 74, 90, 92, False, "Sans" ) )


        self.goback = wx.Button( self, wx.ID_ANY, "Backward or Arrow down",wx.Point(275,185), wx.Size( 200,50 ), 0 )
        self.goback.SetFont( wx.Font( 13, 74, 90, 92, False, "Sans" ) )

        self.turnleft = wx.Button( self, wx.ID_ANY, "Left or Arrow left", wx.Point(155,155), wx.Size( 200,50 ), 0 )
        self.turnleft.SetFont( wx.Font( 13, 74, 90, 92, False, "Sans" ) )

        self.turnright = wx.Button( self, wx.ID_ANY, "Right or Arrow right",wx.Point(395,155), wx.Size( 200,50 ), 0 )
        self.turnright.SetFont( wx.Font( 13, 74, 90, 92, False, "Sans" ) )

        self.m_bitmap5 = wx.StaticBitmap( self, wx.ID_ANY, wx.Bitmap("voiture.png", wx.BITMAP_TYPE_ANY ), wx.Point(349,425), wx.Size( 100,200 ), 0 )

        self.ready = wx.ToggleButton( self, wx.ID_ANY, "Ready to Go",wx.Point(205,210), wx.Size( 100,50 ), 0 )
        self.ready.SetFont( wx.Font( 13, 74, 90, 92, False, "Sans" ) )
        self.ready.SetValue(False)
                
        self.close = wx.Button( self, wx.ID_ANY, "Close", wx.Point(445,210), wx.Size( 100,55 ), 0 )
        self.close.SetFont( wx.Font( 11, 74, 90, 92, False, "Sans" ) )
                
        self.teamlog = wx.StaticBitmap( self, wx.ID_ANY, wx.Bitmap( "logo-projet.png", wx.BITMAP_TYPE_ANY ),  wx.Point(220,0), wx.DefaultSize, 0 )

        self.timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.OnTimer)

        self.timer.Start(1000)    # 1 second interval
         
        self.Bind(wx.EVT_CHAR_HOOK, self.OnKeyDown)
        self.Centre()

        self.Bind( wx.EVT_CLOSE, self.Closeonme )
        self.gofoward.Bind( wx.EVT_BUTTON, self.Gofoward )
        self.goback.Bind( wx.EVT_BUTTON, self.Goback )
        self.turnleft.Bind( wx.EVT_BUTTON, self.Left )
        self.turnright.Bind( wx.EVT_BUTTON, self.Right )
        self.close.Bind( wx.EVT_BUTTON, self.Closeon )
        self.ready.Bind(wx.EVT_TOGGLEBUTTON, self.OnToggleClick)
        self.SetFocus()
   
    def OnKeyDown(self, event):
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_UP:
            self.Gofoward(True)
        else:
            if keycode == wx.WXK_DOWN:
                self.Goback(True)
            else:
                if keycode == wx.WXK_RIGHT:
                    self.Right(True)
                else:
                    if keycode == wx.WXK_LEFT:
                        self.Left(True)
                    else:
                        if keycode == wx.WXK_SPACE:
                            g_var.write_mvt_cmd("MOV" + "stop")
                            self.animateUP(False)
                            self.animateDOWN(False)
                            self.animateLEFT(False)
                            self.animateRIGHT(False)
                            
                        else:
                            if keycode == wx.WXK_SHIFT:
                                g_var.write_mvt_cmd("STE" + "stop")
                                self.animateLEFT(False)
                                self.animateRIGHT(False)
                            else:
                                event.Skip()
   

    def OnToggleClick(self,state):
        state = self.ready.GetValue()
        #print("-----on toggle click ------")
        self.animateUP(False)
        self.animateDOWN(False)
        self.animateLEFT(False)
        self.animateRIGHT(False)
         

    def Pauseon(self,event):
        g_var.write_mvt_cmd("STE" + "stop")
        g_var.write_mvt_cmd("MOV" + "stop")
        self.animateUP(False)
        self.animateDOWN(False)
        self.animateLEFT(False)
        self.animateRIGHT(False)
        #self.animateEMERGENCY(False)
                    

    def animateLEFT( self, state ):
        gif_fname="run2.gif"
        anim_l = wx.adv.Animation()
        anim_l.LoadFile(gif_fname,wx.adv.ANIMATION_TYPE_GIF)
        gif_l = wx.adv.AnimationCtrl(self, id=wx.ID_ANY, anim=anim_l, pos=(149,425),size=wx.DefaultSize, name="Clignotte")
        if state:
            gif_l.Play()
        else:
            gif_l.Stop()
             
    def animateRIGHT(self, state):
        gif_fname="run2.gif"
        anim_r = wx.adv.Animation()
        anim_r.LoadFile(gif_fname,wx.adv.ANIMATION_TYPE_GIF)
        gif_r = wx.adv.AnimationCtrl(self, id=wx.ID_ANY, anim=anim_r, pos=(400,425),size=wx.DefaultSize, name="Clignotte")
        if state:
            gif_r.Play()
        else:
            gif_r.Stop()

    def animateUP( self, state):
        gif_fname="run2.gif"
        anim_u = wx.adv.Animation()
        anim_u.LoadFile(gif_fname,wx.adv.ANIMATION_TYPE_GIF)
        gif_u = wx.adv.AnimationCtrl(self, id=wx.ID_ANY, anim=anim_u, pos=(275,275),size=wx.DefaultSize, name="Clignotte")
        if state:
            gif_u.Play()
        else:
            gif_u.Stop()

    def animateDOWN( self, state):
        gif_fname="run2.gif"
        anim_d = wx.adv.Animation()
        anim_d.LoadFile(gif_fname,wx.adv.ANIMATION_TYPE_GIF)
        gif_d = wx.adv.AnimationCtrl(self, id=wx.ID_ANY, anim=anim_d, pos=(275,525),size=wx.DefaultSize, name="Clignotte")
        if state:
            gif_d.Play()
        else:
            gif_d.Stop()
            
    #A ne pas toucher
    def animateEMERGENCY(self, state):
        gif_fname="stop.gif"
        anim_eme = wx.adv.Animation()
        anim_eme.LoadFile(gif_fname,wx.adv.ANIMATION_TYPE_GIF)
        gif_eme = wx.adv.AnimationCtrl(self, id=wx.ID_ANY, anim=anim_eme, pos=(500,275),size=wx.DefaultSize, name="Clignotte2")
        if state:
            gif_eme.Play()
        else:
            gif_eme.Stop()
            
    def OnTimer(self, event):
        emer = g_var.read_emergency()
        if emer ==True:
            self.animateEMERGENCY(True)
        else:
            self.animateEMERGENCY(False)

    def Gofoward( self, event):
        if self.ready.GetValue():
            self.animateUP(True)
            self.animateDOWN(False)
            self.animateLEFT(False)
            self.animateRIGHT(False)
            g_var.write_mvt_cmd("MOV"+"forward")

    def Goback( self, event):
        if self.ready.GetValue():
            self.animateUP(False)
            self.animateDOWN(True)
            self.animateLEFT(False)
            self.animateRIGHT(False)
            g_var.write_mvt_cmd("MOV"+"backward")

    def Left( self, event):
        if self.ready.GetValue():
            self.animateLEFT(True)
            self.animateRIGHT(False)
            self.animateUP(False)
            self.animateDOWN(False)
            g_var.write_mvt_cmd("STE" + "left")

    def Right( self, event):
        if self.ready.GetValue():
            self.animateLEFT(False)
            self.animateRIGHT(True)
            self.animateUP(False)
            self.animateDOWN(False)
            g_var.write_mvt_cmd("STE" + "right")


    def Closeon( self, event):
        self.Pauseon(True)
        self.Close(True)
        
    def Closeonme(self, event):
        self.Destroy()

class GUI(Thread):
    global g_var

    def __init__(self):
        Thread.__init__(self)
        self.app = wx.App()
        self.window = MyFrame(None)
    
    def run(self):
        try:
            self.window.Show(True)
        except(KeyboardInterrupt):
            return


global first_value
global nb_frame
global all_frame
global size_of_frame
global t1
g_var = global_variable()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.43.39", 1245))
send_msg = send()
receive_msg = receive()
g = GUI()
#eme = Test_emergency()
fig = plt.figure(figsize=(8,8))
ax1= fig.add_subplot(1,1,1)

first_value = 1
nb_frame = 1
t1 = time.time()

send_msg.start()
receive_msg.start()
g.start()
#eme.start()
ani = animation.FuncAnimation(fig,dynamic_map,interval=50)
try:
    plt.show()
except(KeyboardInterrupt):
    time.sleep(10)
    receive_msg.join()
    send_msg.join()
    g.join()
    #eme.join()












