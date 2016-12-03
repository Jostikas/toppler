import serial
import time
import threading
import re
import MagicalConstants

'''
    Important notes:
    Every function, that has a parameter for robotID, takes in a NON-REMAPPED ID value! ( except for getRemappedIDs() )
'''

class Interface:

    def dbg(self, text):
        """
            Prints to the program output with a prefix to indicate that it
            was printed from the Electronics Interface (E-I) class
            :param text: the text that should be printed
            :return: nothing
        """
        print("E-I: " + str(text))

    def __init__(self, port):
        self.dbg("ELECTRONICS STARTING WITH PORT " + port)
        self.raw = RawElectronics()
        self.raw.init(port)

    def __del__(self):
        self.raw.closeConnection()

    def moveUpperRobot(self, robotID, speed):
        """
            moves the upper robots sideways
            :param robotID: the ID of the robot that is needed to be moved \
                the robot ID is either 0 (nearer to the starting position) \
                or 1 (further away from the starting position)
            :param speed: the speed at which the robot should be moved \
                a positive speed value moves it away from the starting position \
                a negative speed value moves it back to its original position
            :return: nothing
        """
        speedToSet = speed
        id = 2
        # map input ID of [0 to 1] and [1 to 2], also reverse the motor direction of one robot
        # I have absolutely no idea, why it is needed, but it works! :)
        if(robotID == 0):
            speedToSet *= -1
            id = 1

        if (robotID == 0 or robotID == 1):
            cmd = "t" + str(id) + ":sd" + str(speedToSet) + "\n"
            self.raw.serialWrite(cmd)
            self.dbg("Sent: " + cmd)
        else:
            self.dbg("Unknown ID entered to moveUpperRobot(): " + str(robotID))

    def moveCrane(self, speed):
        """
            :param speed: moves the crane up or down with a specified speed \
                a positive speed value moves it upwards (max 190) \
                a negative speed value moves it downwards (minimum -190)
            :return: nothing
        """
        self.raw.serialWrite("t3:sd" + str(speed) + "\n")


    def meltFishingLine(self, time):
        """
            tries to melt the fishing line with a constant power output for a specified time
            :param time: a positive integer that represents the time in some unknown units \
            * a value that should be used for testing should be somewhere about 150 \
            * the time scales linearly so 300 would be twice the time
            :return: nothing
        """
        self.raw.serialWrite("l1m" + str(time) + "\n")

    def driveRobot(self, robotID, speed = 0, arcSize = 0):
        """
            This function makes a specified robot move in a wantd direction
            :param robotID: the ID of the robot you want to move
            :param speed: the speed at which you want the robot to move (it is \
                    probably going to be a binary value, meaning that it is either stopped or moving)
            :param arcSize: this value lets you fine-tune the course of the robot.
            :return: nothing
        """
        # TODO this is gonna be awful :)
        # TODO Make a lookup table containing some values for moving in a straight line
        # TODO Implement software breaking on the robots (not in python!)

        leftBoost = 0
        rightBoost = 0
        if(arcSize > 0):
            rightBoost = arcSize
        else:
            leftBoost = arcSize * -1

        self.dbg("leftBoost: "  + str(leftBoost) + " rightBoost: " + str(rightBoost))

        if(speed == 0):
           self.setRobotMotorSpeeds(robotID, 0, 0)
        elif(robotID == 0):
            if (speed > 0):
                # move the robot forward
                self.setRobotMotorSpeeds(robotID, MagicalConstants.ROBOT_2_MOVE_FORWARD_LEFT_MOTOR_SPEED + leftBoost,
                                         MagicalConstants.ROBOT_2_MOVE_FORWARD_RIGHT_MOTOR_SPEED + rightBoost)
            else:
                # move the robot backward
                self.setRobotMotorSpeeds(robotID, MagicalConstants.ROBOT_2_MOVE_BACKWARD_LEFT_MOTOR_SPEED - leftBoost,
                                         MagicalConstants.ROBOT_2_MOVE_BACKWARD_RIGHT_MOTOR_SPEED - rightBoost)
        elif(robotID == 1):
            if (speed > 0):
                # move the robot forward
                self.setRobotMotorSpeeds(robotID, MagicalConstants.ROBOT_1_MOVE_FORWARD_LEFT_MOTOR_SPEED + leftBoost,
                                         MagicalConstants.ROBOT_1_MOVE_FORWARD_RIGHT_MOTOR_SPEED + rightBoost)
            else:
                # move the robot backward
                self.setRobotMotorSpeeds(robotID, MagicalConstants.ROBOT_1_MOVE_BACKWARD_LEFT_MOTOR_SPEED - leftBoost,
                                         MagicalConstants.ROBOT_1_MOVE_BACKWARD_RIGHT_MOTOR_SPEED - rightBoost)
        else:
            self.dbg("Unknown robot ID entered to driveRobot(): " + str(robotID))

        self.dbg("Liigutaks maapealset autot, aga see pole veel implementeeritud :(")

        #self.raw.serialWrite("")

    def rotateRobot(self, robotID, amount, shouldMoveForward = 0):
        """
        This function is needed if precise turning of a robot is needed. While it is not accurate
        (500 will not turn the robot the same amount each time it is used), it is still the only
        way the robot can be turned while being stationary.
        A good starting value would be somewhere between 500 and 700, which should turn the robot
        about 10-20 degrees. As it does not scale linearly, a value of 300 would do nothing and a
        value of 2000 would make the robot spin more than a full revoultion

        :param robotID: The ID of the robot that is needed to be moved
        :param amount: The amount of rotation wanted from the robot. \
                Negative values rotate the robot counter-clockwise (TODO untested), \
                positive clockwise. NB! This value does not scale linearly!!!
        :return: nothing
        """

        # TODO ##################################################################################
        # TODO Maybe make a look-up table and then pick the values from there in order to turn  #
        # TODO both ways the same amount.                                                       #
        # TODO Still, as it won't most probably scale linearly, it could be quite pointless ... #
        # TODO ##################################################################################

        speedWhileTurning = 2000
        if(shouldMoveForward != 0):
            speedWhileTurning *= -1
            amount *= -1

        # map input ID of [0 to 1] and [1 to 2]
        id = self.getRemappedRobotIDs(robotID)

        if (robotID == 0 or robotID == 1):
            if(amount < 0):
                amount *= -1
                self.setMotorBuffers(id, 0, speedWhileTurning * -1)
            else:
                self.setMotorBuffers(id, speedWhileTurning * -1, 0)
            cmd = "l" + str(id) + "t" + str(amount) + "\n"
            self.raw.serialWrite(cmd)
            self.dbg("Sent: " + cmd)
        else:
            self.dbg("Unknown ID entered to rotateRobot(): " + str(robotID))

    """
        Sets the motor speeds of the robot indicated by ID (not remapped)
    """
    def setRobotMotorSpeeds(self, robotID, l, r):
        self.setMotorBuffers(robotID, l, r)
        self.raw.serialWrite("l" + str(self.getRemappedRobotIDs(robotID)) + "s\n")

    """
        Sets the motor buffers of the robot according to the motor placement and stuff like that.
        It then sends those values to the robot. NB! It does not start any action (moving, rotating, etc.)
    """
    def setMotorBuffers(self, robotID, left, right):
        id = self.getRemappedRobotIDs(robotID)
        cmd = ""
        if(id == 1):
            #this is the BLUE robot that isn't under the robot with the NUC
            print("###FIRST")
            cmd += "l" + str(id) + "l" + str(-right) + "\n"
            cmd += "l" + str(id) + "r" + str(left) + "\n"
        else:
            print("###SECOND")
            # this is the ORANGE robot near the NUC, because the ID is already REMAPPED!!!
            cmd += "l" + str(id) + "l" + str(right) + "\n" #because fuck logic :)
            cmd += "l" + str(id) + "r" + str(-left) + "\n"
        print("id: " + str(robotID) + " sent: " + cmd.replace("\n", "[\\n]"))
        self.raw.serialWrite(cmd)

    """
        As the elctronics programs use different IDs, these must be converted.
        This function converts the main code IDs to the electronics equivalents
    """
    def getRemappedRobotIDs(self, robotID):
        # map input ID of [0 to 2] and [1 to 1]
        if (robotID == 0):
            print("0 TO 2")
            return 2
        elif(robotID == 1):
            print("1 TO 1")
            return 1
        else:
            self.dbg("Invalid robotID entered to getRemappedMotorIDs(). It was " + str(robotID))
            return 1 # the default value

    def setStartCallback(self, functionToCall):
        self.raw.startCallback = functionToCall

    def setStopCallback(self, functionToCall):
        self.raw.stopCallback = functionToCall


class RawElectronics:

    ser = 0
    startCallback = 0
    stopCallback = 0

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

                #self.dbg("got:" + recv)
                if("<event:1>" in recv):
                    if(self.startCallback != 0):
                        self.startCallback()
                if("<event:0>" in recv):
                    if(self.stopCallback != 0):
                        self.stopCallback()
            time.sleep(0.1)
        self.dbg("ELECTRONICS MAINLOOP HAS STOPPED!")

    def __del__(self):
        self.closeConnection()

    def closeConnection(self):
        if self.isConnectionActive():
            self.dbg("closing connections ...")
            self.serialWrite("9:led9\n")
            ser.close()
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