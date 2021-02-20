import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

#arm-parameters
length =500 #cm
mass=1 #kg
InertiaY= 3 #kgm^2

#thrust eqn Thrust= 4E-07*rpm**2 - 0.0016*rpm + 3.0106

#initial conditions

Rpm_Left=5000
Rpm_Right=5000

Thrust_Right=0
Thrust_Left=0

acelX=0 #m/s^2, right (+ve)
acelZ=-9.81 #m/s^2, up (+ve)
alphaY=0 #deg/s^2, CW (+ve)

veloX=0 #m/s
veloZ=0
omegaY=0 #deg/s

cg_Z=0 #m
cg_X=0
thetaY=0 #deg

PowerAngle=90-thetaY
PowerAngle=math.radians(PowerAngle)

#time variable = t
#compute constant =c =1/(timescale*cc)
ComputeConst=(1/500)

#outputlists
Rpm_Right_list=[]
Rpm_Left_list=[]
Thrust_Left_list=[]
Thrust_Right_list=[]
cg_X_list=[]
cg_Z_list=[]
thetaY_list=[]
veloZ_list=[]

#tracker
t_list=[] #timexdomain
c_list=[] #computedomain
i=0 #general purpose counter

#altitude - PID
Alt_P=20
Alt_I=2
Alt_D=30

#Roll - PID
Setpoint=2.5#degs
Roll_P=4
Roll_D=12
Roll_DD=15

#OutputProbe
Val1_list=[]
Val2_list=[]
Val3_list=[]
Val4_list=[]
Val5_list=[]
Val6_list=[]

for t in range (30):

    for c in range(500):
        i = i+1
        #precalcs:

        # Magnitudes
        cg_X = cg_X + veloX * ComputeConst
        cg_Z = cg_Z + veloZ * ComputeConst
        thetaY = thetaY + omegaY* ComputeConst

        #first der.
        veloX=veloX+acelX* ComputeConst
        veloZ=veloZ+acelZ* ComputeConst
        omegaY=omegaY+alphaY* ComputeConst

        #livecalcs:
        PowerAngle=90-thetaY
        PowerAngle=math.radians(PowerAngle)

        #propellers can't generate downforce
        if (Rpm_Left*Rpm_Right>0):
            Thrust_Right=4E-07*Rpm_Right**2 - 0.0016*Rpm_Right + 3.0106
            Thrust_Left=4E-07*Rpm_Left**2 - 0.0016*Rpm_Left + 3.0106
        else:
            Thrust_Left=0;
            Thrust_Right=0;

        ForceRightZ=math.sin(PowerAngle)*(Thrust_Right)
        ForceRightX=math.cos(PowerAngle)*(Thrust_Right)
        ForceLeftZ=math.sin(PowerAngle)*(Thrust_Left)
        ForceLeftX=math.cos(PowerAngle)*(Thrust_Left)

        TorqueY=(ForceLeftZ-ForceRightZ)*length/2000

        ForceZ=ForceRightZ+ForceLeftZ-(9.81*mass)
        acelZ=ForceZ/(mass*9.81)
        acelX=(ForceLeftX+ForceRightX)/(mass*9.81)
        alphaY=TorqueY/InertiaY

        #Altitude Velocity PID-Gainz
        Rpm_Left = Rpm_Left-cg_Z*Alt_I
        Rpm_Right = Rpm_Right-cg_Z*Alt_I
        Rpm_Left = Rpm_Left-veloZ *Alt_P
        Rpm_Right = Rpm_Right-veloZ *Alt_P
        Rpm_Left = Rpm_Left - acelZ * Alt_D
        Rpm_Right = Rpm_Right - acelZ *Alt_D

        #probes
        Val3_list.append(acelZ * Alt_D)
        Val1_list.append(cg_Z * Alt_I)
        Val2_list.append(veloZ * Alt_P)

        #Roll PID-Gainz
        if (thetaY != Setpoint):
            Rpm_Left=Rpm_Left-((thetaY-Setpoint)*Roll_P)
            Rpm_Right=Rpm_Right+((thetaY-Setpoint)*Roll_P)
            Rpm_Left = Rpm_Left -((omegaY ) * Roll_D)
            Rpm_Right = Rpm_Right+ ((omegaY) * Roll_D)
            Rpm_Left = Rpm_Left - ((alphaY) * Roll_DD)
            Rpm_Right = Rpm_Right + ((alphaY) * Roll_DD)
            #probes
            Val4_list.append((thetaY - Setpoint) * Roll_P)
            Val5_list.append((omegaY ) * Roll_D)
            Val6_list.append((alphaY) * Roll_DD)
        c_list.append(i)



    #output
    Rpm_Right_list.append(Rpm_Right)
    Rpm_Left_list.append(Rpm_Left)
    Thrust_Left_list.append(Thrust_Left)
    Thrust_Right_list.append(Thrust_Right)
    cg_X_list.append(cg_X)
    cg_Z_list.append(cg_Z)
    thetaY_list.append(thetaY)
    t_list.append(t)
    veloZ_list.append(veloZ)


plt.figure(3)
plt.plot(c_list,Val1_list,"r",c_list,Val2_list,"b",c_list,Val3_list,"g")
#plt.figure(2)
#plt.plot(t_list, veloZ_list,"b")

#plt.figure(1)
#plt.plot(c_list,Val4_list,"r",c_list,Val5_list,"g",c_list,Val6_list,"b")
#plt.figure(2)
#plt.plot(t_list,thetaY_list,"p")

plt.figure(4)
plt.plot(t_list,Thrust_Left_list,"b--",t_list,Thrust_Right_list,"g--")

plt.figure(5)
plt.plot(cg_Z_list,cg_X_list,"b")

#plt.figure(5)
#plt.plot(t_list,Rpm_Left_list,"r--")
#print(Rpm_Left_list)


plt.show()