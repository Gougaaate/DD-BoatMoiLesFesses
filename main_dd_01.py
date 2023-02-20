import time
import numpy as np
from imu9_driver_v2 import *
from tc74_driver_v2 import *
from encoders_driver_v2 import *
from arduino_driver_v2 import *
from gps_driver_v2 import *
#from scipy import sawtooth




fichier = open("data.txt", "a")
fichier.write("bonjour")


class Bateau:
    def __init__(self):
        self.IMU = Imu9IO()
        self.A = np.array([[-71902173.91304347,   8489130.43478261 , 13489130.43478261],
 [ 15163043.47826087,  71054347.82608695,   9532608.69565217],
 [ -3206521.73913043 ,  1032608.69565217, -72597826.08695652]])
        self.b = np.array([ [-560.5] , [1583.5] ,[-6097.5]])


        self.temp = TempTC74IO()

        dt = 2
        self.enc = EncoderIO()
        self.enc.set_older_value_delay_v2(10*dt)

        self.ard = ArduinoIO()
        self.gps = GpsIO()
        self.gps.set_filter_speed("0.4")
        self.gps.get_filter_speed()
        self.gps.set_filter_speed("0")
        self.gps.get_filter_speed()

        print("it's raining men, Hallelujah")


    def calibrate_MAG(self) :
        """
        fonction à utiliser une fois pour calibrer le capteur GPS
        :return: nothing
        """
        x1, xm1, x2, x3 = 0, 0, 0, 0
        input("get measure? tap enter")
        x1 = self.IMU.read_mag_raw()
        input("get measure? tap enter")
        xm1 = self.IMU.read_mag_raw()
        input("get measure? tap enter")
        x2 = self.IMU.read_mag_raw()
        input("get measure? tap enter")
        x3 = self.IMU.read_mag_raw()


        x1 = np.array(x1)
        xm1 = np.array(xm1)
        x2 = np.array(x2)
        x3 = np.array(x3)

        beta = 46*10**-6

        self.b = (-x1 - xm1)/2
        X = np.zeros((3, 3))
        X[:, 0] = x1 + self.b
        X[:, 1] = x2 + self.b
        X[:, 2] = x3 + self.b
        self.A = 1/beta*X

        print(self.A, self.b)

<<<<<<< HEAD
    def get_angle(self):
        y = np.linalg.inv(self.A) @ (
            np.array(self.IMU.read_mag_raw()).reshape(3, 1) + self.b)
        return 180 / np.pi * np.arctan2(
            y[1, 0], y[0, 0])  # retourne le cap actuel du bateau

    def test(self):  # to test calibration
=======

    def get_angle(self):
        y = np.linalg.inv(self.A)@(np.array(self.IMU.read_mag_raw()).reshape(3, 1) + self.b)
        print(y[0], y[1])
        return -180/np.pi*np.arctan2(y[1, 0], y[0, 0]) #retourne le cap actuel du bateau



    def test(self): #to test calibration
>>>>>>> 185a32732aad24012e3a95c7a351c8221b465664
        for k in range(50):
            print(self.get_angle())
            time.sleep(2)

<<<<<<< HEAD
    def omega(self, k, dt):
        # a1, a2 = self.enc.read_packet()
        a, b = self.enc.get_last_and_older_values_v2()
        a.split(",")
        b.split(",")
        odo_left0 = a[3]
        odo_left1 = b[3]
        odo_right0 = a[4]
        odo_right1 = b[4]
        delta_odo_left = eval(odo_left0) - eval(odo_left1)
        delta_odo_right = eval(odo_right0) - eval(odo_right1)
=======
>>>>>>> 185a32732aad24012e3a95c7a351c8221b465664




    def omega2(self):
        #time.sleep(5.0)
        delta_t = 3.0
        n_t = int(round(delta_t * 10))
        self.enc.set_older_value_delay_v2(n_t)

        st1, st0 = self.enc.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(',')).astype(np.float)
        data_encoders1 = np.array(st1.split(',')).astype(np.float)

        timeAcq0 = data_encoders0[0]/10.0
        sensLeft0 = data_encoders0[2]
        sensRight0 = data_encoders0[1]
        posLeft0 = data_encoders0[4]
        posRight0 = data_encoders0[3]

        timeAcq1 = data_encoders1[0]/10.0
        sensLeft1 = data_encoders1[2]
        sensRight1 = data_encoders1[1]
        posLeft1 = data_encoders1[4]
        posRight1 = data_encoders1[3]

        delta_t = timeAcq1 - timeAcq0
        rpmL = abs(delta_odo(posLeft1, posLeft0)/8.0/delta_t*60.0)
        rpmR = abs(delta_odo(posRight1, posRight0)/8.0/delta_t*60.0)

        #print("RPM Left", rpmL, "RPM Right", rpmR)

        return abs(rpmL), abs(rpmR)





    def choosew(self, wbarbar, deltawbar):
        wbar1, wbar2 = wbarbar, wbarbar + abs(deltawbar)
        if deltawbar < 0:
            #print("go right")
            return wbar2, wbar1
        elif deltawbar >= 0:
            #print("go left")
            return wbar1, wbar2





    def follow_one_cap(self, psibar, T):
        """
        fonction to follow a direction psi
        :param psi: is an angle
        :return: nothing
        """
<<<<<<< HEAD
        dt = 0.1
        K11, K12, K21, K22, K3 = 1, 1, 0.01, 0.01, 1
        z1, z2 = 0, 0
        # wbar1, wbar2 = 0, 0
=======
        dt = 0.5
        K11, K12, K21, K22, K3 = 0., 0., 0.06, 0.06, 650
        z1, z2 = 70, 70
        fichier.write("################ Nouvelle Run ###################\n\n")
>>>>>>> 185a32732aad24012e3a95c7a351c8221b465664

        k = 0
        T0 = time.time()
        while time.time() - T0 < T:
            k += 1
            t0 = time.time()

            psi = self.get_angle()
            deltawbar = K3*sawtooth(np.pi*psibar/180 - np.pi*psi/180) #sawtooth prend des radians
            #deltawbar = 0 #provisoirement
            wbarbar = 3000 #consigne pivot 
            wbar1, wbar2 = self.choosew(wbarbar, deltawbar) #détermine si on va à gauche ou à droite
            w1, w2 = self.omega2() #retourne les mesures des rpm
            e1, e2 = wbar1 - w1, wbar2 - w2

            z1 += e1*dt #termes intégrateurs
            z2 += e2*dt
            u1 = K11*e1 + K21*z1
            u2 = K12*e2 + K22*z2
            
            print("###################################")
            print("rpmL : ", w1, "rpmR : ", w2)
            print("omega bar : ", wbar1, wbar2)
            print("z : ", z1, z2)
            print("u : ", u1, u2)
            print("psi : ", psi)
            fichier.write("###################################\n")
            fichier.write("rpmL : " + str(w1) + "rpmR : " + str(w2) + "\n")
            fichier.write("z : " + str(z1) + str(z2) + "\n")
            fichier.write("u : " + str(u1) + str(u2) + "\n")
            fichier.write("psi : " + str(psi) + "\n")


            self.ard.send_arduino_cmd_motor(u1, u2)

            t1 = time.time()
            if t1 - t0 < dt:
                time.sleep(dt - (t1 - t0))

        self.ard.send_arduino_cmd_motor(0, 0) #on coupe les moteurs 



    def follow_several_caps(self, psibarL, TL):
        """
        fonction to follow a direction psi
        :param psibarL: is a list with all the caps
        :param TL : is a list with the direction times
        :return: nothing
        """
        for cap in range(len(psibarL)):
            self.follow_one_cap(psibarL[cap], TL[cap])
            time.sleep(1) #petite pause entre deux caps
            print("ok bb je passe au cap suivant ;) ;)")






    def get_lat_lon(self):
        msg, gps_data_string = self.gps.read_gll_non_blocking()
        while not msg:
            msg, gps_data_string = self.gps.read_gll_non_blocking()
        lat, lon = cvt_gll_ddmm_2_dd(gps_data_string)
        print(lat, lon)
        print("GPS:", gps_data_string)
        return lat, lon




    def get_psibar(self, p, phat, vhat, nhat):
        dhat = vhat - 2 * nhat @ np.transpose(nhat)@(p - phat)
        psibar = np.arctan2(dhat[1, 0], dhat[0, 0])
        return -180 * psibar / np.pi





    def follow_line(self, phat, ptilde): #suit juste une ligne
        """
        phat et ptilde sont des coordonnées en cartésien (en vecteurs colonnes)
        """

        dt1, dt2, dt3 = 0.5, 1, 3
        t1 = t2 = t3 = time.time() - 3
        K11, K12, K21, K22, K3, K4  = 0., 0., 0.05, 0.06, 650, 40
        z1, z2 = 70, 70
        psibar = 0
        psi = 0
        wbar1, wbar2 = 0, 0
        wbarbar = 3000 #consigne pivot
        nhat, vhat = get_nhat(phat, ptilde)


        cond = np.array([[0], [0]])
        while np.linalg.norm(cond - ptilde) > 6: #tant qu'éloigné de la balise de plus de 6 mètres
            t0 = time.time()

            if time.time() - t3 > dt3:
                lat, lon = self.get_lat_lon()
                p = convert_coordinates(lat, lon)
                psibar = self.get_psibar(p, phat, K4 * vhat, nhat)
                t3 = time.time()
                print("psibar recalculé")


            if time.time() - t2 > dt2:
                psi = self.get_angle()
                deltawbar = K3*sawtooth(np.pi*psibar/180 - np.pi*psi/180) #sawtooth prend des radians
                #deltawbar = 0 #provisoirement
                wbar1, wbar2 = self.choosew(wbarbar, deltawbar) #détermine si on va à gauche ou à droite
                t2 = time.time()
                print("deltawbar recalculé")

            w1, w2 = self.omega2() #retourne les mesures des rpm
            e1, e2 = wbar1 - w1, wbar2 - w2

            z1 += e1*dt1 #termes intégrateurs
            z2 += e2*dt1
            u1 = K11*e1 + K21*z1
            u2 = K12*e2 + K22*z2
            
            print("###################################")
            print("rpmL : ", w1, "rpmR : ", w2)
            print("omega bar : ", wbar1, wbar2)
            #print("z : ", z1, z2)
            #print("u : ", u1, u2)
            print("psi : ", psi)
            print("psibar : ", psibar)


            self.ard.send_arduino_cmd_motor(u1, u2)

            t1 = time.time()
            if t1 - t0 < dt1:
                print("c bon")
                time.sleep(dt1 - (t1 - t0))

        self.ard.send_arduino_cmd_motor(0, 0) #on coupe les moteurs
        print("c'est bon, je suis au point GPS demandé")





    """
    def v2(self, psibar, T):
        
        dt = 0.1
        K11, K12, K21, K22, K3 = 0., 0., 0.1, 0.1, 300
        z1, z2 = 70, 70
        fichier.write("################ Nouvelle Run ###################\n\n")

        k = 0
        T0 = time.time()
        while time.time() - T0 < T:
            k += 1
            t0 = time.time()

            psi = self.get_angle()
            deltawbar = K3*sawtooth(np.pi*psibar/180 - np.pi*psi/180) #sawtooth prend des radians
            #deltawbar = 0 #provisoirement
            wbarbar = 1000 #consigne pivot 
            wbar1, wbar2 = self.choosew(wbarbar, deltawbar) #détermine si on va à gauche ou à droite
            
            t_test = time.time()
            w1, w2 = wbarbar, wbarbar
            while abs(wbar1 - w1) > 80 or abs(wbar2 - w2) > 80:
                u1, u2 = 0, 0
                w1pro, w2pro = self.omega2()
                if abs(wbar1 - w1) > 80:
                    w1 = w1pro
                    e1 = wbar1 - w1
                    z1 += e1*dt
                    u1 = K11*e1 + K21*z1

                if abs(wbar2 - w2) > 80:
                    w2 = w2pro
                    e2 = wbar2 - w2
                    z2 += e2*dt
                    u2 = K12*e2 + K22*z2

                self.ard.send_arduino_cmd_motor(u1, u2)

            
            print("###################################")
            print("temps de regulation moteur : ", time.time() - t_test)
            print("rpmL : ", w1, "rpmR : ", w2)
            print("omega bar : ", wbar1, wbar2)
            print("z : ", z1, z2)
            print("u : ", u1, u2)
            print("psi : ", psi)
            fichier.write("###################################\n")
            fichier.write("rpmL : " + str(w1) + "rpmR : " + str(w2) + "\n")
            fichier.write("z : " + str(z1) + str(z2) + "\n")
            fichier.write("u : " + str(u1) + str(u2) + "\n")
            fichier.write("psi : " + str(psi) + "\n")



            t1 = time.time()
            if t1 - t0 < dt:
                time.sleep(dt - (t1 - t0))

        self.ard.send_arduino_cmd_motor(0, 0) #on coupe les moteurs 
    """



def delta_odo(odo1, odo0):
    '''
    computes the difference between 2 encoder values
    '''
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo



def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))




def cvt_gll_ddmm_2_dd(st): #sert à convertire les données NMEA en coord lat lon
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat / 100))
    olon = float(int(ilon / 100))
    olat_mm = (ilat % 100) / 60
    olon_mm = (ilon % 100) / 60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat, olon



def convert_coordinates(lat, lon):
        roh = 6371000
        lat_ponton = 48.199024
        lon_ponton = -3.014790
        xtilde = roh * np.cos(lat) * (lon - lon_ponton)
        ytilde = roh * (lat - lat_ponton)
        return np.array([[xtilde], [ytilde]]) #retourne un vecteur colonne



def get_nhat(phat, ptilde): #phat point de départ, ptilde point d'arrivée
        """
        get_line renvoie nhat, le vecteur orthogonal à la ligne
        """
        u = ptilde - phat
        unorm = np.linalg.norm(u)

        vhat = u/unorm
        nhat= np.array([[-u[1, 0]], [u[0, 0]]]) / unorm
        return nhat, vhat





if __name__ == "__main__":

    p_bouee_ouest = convert_coordinates(48.199038, -3.015807)
    p_bouee_nord = convert_coordinates(48.199817, -3.015603)
    print(p_bouee_ouest)
    nhat, vhat = get_nhat(np.array([[0], [0]] ), p_bouee_ouest)
    print(nhat)
    print(vhat)


    bat = Bateau()
<<<<<<< HEAD
    # bat.calibrate_MAG()
    # bat.test()
    bat.follow_one_cap(0)
=======

    psibarL = [0, 90, 180, 270] #valeurs en degrés
    TL = [40, 30, 30, 30]

    #bat.calibrate_MAG()
    #bat.test()
    #bat.follow_one_cap(0, 60)
    #bat.follow_several_caps(psibarL, TL)
    #bat.follow_line(np.array([[0], [0]]), p_bouee_ouest)
    bat.follow_line(p_bouee_ouest, p_bouee_nord)
    #bat.v2(0, 60)
    #bat.get_lat_lon()

fichier.close()


>>>>>>> 185a32732aad24012e3a95c7a351c8221b465664
