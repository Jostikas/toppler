import serial
import time
import threading
import re

class Interface:

    def dbg(self, text):
        print("E-I: " + str(text))

    def __init__(self, port):
        self.dbg("ELECTRONICS STARTING WITH PORT " + port)
        self.raw = RawElectronics()
        self.raw.init(port)

    def __del__(self):
        self.raw.closeConnection()

    # moves the upper robots sideways
    # a positive speed value moves it away from the starting position
    # a negative speed value moves it back to its original position
    # the robot ID is either 0 (nearer to the starting position) or 1 (further away from the starting position)
    def moveUpperRobot(self, robotID, speed):
        speedToSet = speed
        id = 2
        if(robotID == 0):
            speedToSet *= -1
            id = 1
        if(robotID == 0  or  robotID == 1):
            cmd = "t" + str(id) + ":sd" + str(speedToSet) + "\n"
            self.raw.serialWrite(cmd)
            self.dbg("Sent: " + cmd)

class RawElectronics:

    ser = 0

    def dbg(self, text):
        print("RAW: " + str(text))

    def isConnectionActive(self):
        return ser.isOpen()


    def serialWrite(self, command):
        if self.isConnectionActive():
            ser.write(bytearray(command, "UTF-8"))


    def pollingMainloop(self):
        self.dbg("polling loop started")
        while self.isConnectionActive():
            waiting = ser.inWaiting()
            # print("waiting", waiting)
            if waiting > 0:
                recv = ser.read(waiting).decode("UTF-8")[:-1]
                self.dbg("got:" + recv)
                # TODO parse the data
            time.sleep(0.1)
        print("ELECTRONICS MAINLOOP HAS STOPPED!")

    def __del__(self):
        self.closeConnectoin()

    def closeConnection(self):
        if self.isConnectionActive():
            self.dbg("closing connections ...")
            self.serialWrite("9:led9\n")
            ser.close();
            self.dbg("Closed!")



    def electronics_ui_mainloop(self):
        while self.isConnectionActive():
            c = input("Sisesta käsk, mis tuleks saata plaadile (q = quit): ")
            if (c == "s"):
                c = "l1l0\nl1r0\nl1s\n"
            elif c.startswith("spd"):
                param = int(c[3:])
                c = "l1l" + str(param) + "\nl1r" + str(-param) + "\nl1s\n"
            elif c.startswith("r"):
                match = re.match(r'r(-?.\d+)', c, re.M | re.I)

                t = 1000
                if(match):
                    t = int(match.group(1))
                if t < 0:
                    t *= -1
                    c = "l1l2000\nl1r2000\nl1t" + str(t) + "\n"
                else:
                    c = "l1l-2000\nl1r-2000\nl1t" + str(t) + "\n"
            elif c.startswith("d"):
                match = re.match(r'd.*?(-?\d+)\s?;\s?(-?\d+)', c, re.M | re.I)
                if match:
                   print("1: " + str(match.group(1)) + " 2: " + match.group(2))
                   c = "l1l" + match.group(1) + "\nl1r" + str(-int(match.group(2))) + "\nl1s\n"
                else:
                   print("no match!")
            elif c == "q":#quit
                print("exiting...")
                self.closeConnection()
                return

            if not c.endswith("\n"):
               c += "\n"
               self.serialWrite(c)
            print("Sent: " + c.replace("\n", "[\\n]"))
            time.sleep(0.4)


    def init(self, portname):
        global ser
        try:
            ser = serial.Serial(portname,
                                baudrate=9600,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS)
            self.serialWrite("9:led18\n")
            pollthread = threading.Thread(target=self.pollingMainloop, args=(), daemon=True)
            #uithread = threading.Thread(target=electronics_ui_mainloop, args=(), daemon=True)

            pollthread.start()
            #uithread.start()


            #_thread.start_new_thread(electronics_polling_mainloop, ())
            #_thread.start_new_thread(electronics_ui_mainloop, ())
            self.dbg("Ühendus loodud!")
            return ser.isOpen()
        except:
            self.dbg("Ei saanud elektroonikaga ühendust! ")
            raise

        return False