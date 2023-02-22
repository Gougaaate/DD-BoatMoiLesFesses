from drivers.gps_driver_v2 import GpsIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
from heading_command import followHeading
import numpy as np
import time
from get_current_heading import getHeadingSimple
from get_motors_RPM import getRPM
gps=GpsIO()

Ainv = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
              [-5700163.24498797, 80185389.18243794, 884174.92923037],
              [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
binv = np.array([-1414.5, 1552.5, -4570.5]).T

def dist_entre_deux_points(a,b):
    phia=a[0]
    lambdaa=a[2]
    phib=b[0]
    lambdab=b[2]
    dist=2*np.arcsin(np.sqrt(np.sin(0.5*(phib-phia))**2+np.cos(phia)*np.cos(phib)*np.sin(0.5*(lambdab-lambdaa))**2))
    return dist

def conversion_manuelle(lat,long):
    rho=6371000
    latm=np.pi/180*48.198943
    longm=np.pi/180*-3.014750
    lat=np.pi/180*lat
    long=np.pi/180*long
    xt=rho*np.cos(lat)*(lat-latm)
    yt=rho*(long-longm)
    return xt,yt

def chooseRPM(wanted_rpm, goal_rpm_diff):
    goal_rpmL, goal_rpmR = wanted_rpm, wanted_rpm + abs(goal_rpm_diff)
    if goal_rpm_diff > 0:
        print("turn right")
        return goal_rpmR, goal_rpmL
    elif goal_rpm_diff <= 0:
        print("turn left")
        return goal_rpmL, goal_rpmR

def sawtooth(x):
    return (x + np.pi) % (
        2 * np.pi) - np.pi  # or equivalently   2*np.arctan(np.tan(x/2))

def suivi_gps(arduino, imu, matA, vecb, vitesse):
    y = np.linalg.inv(matA) @ (np.array(imu.read_mag_raw()) + vecb).T
    #arduino.send_arduino_cmd_motor(0, 0)
    Ax,Ay=conversion_manuelle(48.199508, -3.015295)
    Bx,By=conversion_manuelle(48.199184, -3.015283)
    A=np.array([[Ax],[Ay]])
    B=np.array([[Bx],[By]])
    phi = np.arctan2(By - Ay,Bx - Ax)
    gll_ok, gll_data = gps.read_gll_non_blocking()
    x,y=conversion_manuelle(gll_data[0],gll_data[2])
    XY=np.array([[x],[y]])
    u = XY - A
    att = XY - B
    v = (B - A) / np.linalg.norm(B - A)
    A = np.hstack((u, v))
    e_dist=np.linalg.det(A) #erreur permettant de rejoindre la ligne
    heading =getHeadingSimple(imu,A,B)
    capdes = phi - np.arctan(e_dist)
    file = open("log.txt", "w")
    dt = 0.1  # time step
    K11, K12, K21, K22, K3 = 0.006, 0.04, 0.05, 0.08, 200  # gains
    z1, z2 = 70, 70  # integral terms
    while abs(capdes-heading)>20:
        heading = getHeadingSimple(imu, matA, vecb)  # current heading
        heading_error = capdes - heading
        if heading_error > 70:
            print("turn right")
            command_pwmL, command_pwmR = 60, 0
            rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
            goal_rpmL, goal_rpmR = 0, 0
        elif heading_error < -70:
            print("turn left")
            command_pwmL, command_pwmR = 0, 60
            rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
            goal_rpmL, goal_rpmR = 0, 0
        else:
            goal_rpm_diff = K3 * heading_error  # desired rpm difference
            wanted_rpm = 3000  # pivot rpm value (constant)
            goal_rpmL, goal_rpmR = chooseRPM(
                wanted_rpm, goal_rpm_diff
            )  # choose the rpm of the motors to turn left or right
            rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
            e1, e2 = goal_rpmL - rpmL, goal_rpmR - rpmR  # error terms
            z1 += e1 * dt
            z2 += e2 * dt

            command_pwmL = K11 * e1 + K21 * z1
            command_pwmR = K12 * e2 + K22 * z2

        if command_pwmL > 255:
            command_pwmL = 255
        elif command_pwmL < 0:
            command_pwmL = 0
        if command_pwmR > 255:
            command_pwmR = 255
        elif command_pwmR < 0:
            command_pwmR = 0

        data_to_write = [
            rpmL, rpmR, goal_rpmL, goal_rpmR, command_pwmL, command_pwmR,
            heading, abs(heading_error)
        ]

        for data in data_to_write:
            file.write(str(data) + " ")
        file.write("\n")

        arduino.send_arduino_cmd_motor(command_pwmL, command_pwmR)

    arduino.send_arduino_cmd_motor(0, 0)  # turn off motors
    file.close()

if __name__ == "__main__":
    arduino = ArduinoIO()
    encoder = EncoderIO()
    imu = Imu9IO()
    suivi_gps(arduino, imu, Ainv, binv, 80)