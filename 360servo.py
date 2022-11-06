#!/usr/bin/env python3
# 参考ページURL
# https://jpdebug.com/p/3461698

#FS90R
#データシート
#https://www.switch-science.com/products/7113/

#pinoutコマンドでGPIO番号確認
#sudo pip3 install gpiozero
#または
#sudo apt install python3-gpiozero

import RPi.GPIO as GPIO
import time

class Servo():
    """ 
    gpioNumber:GPIO
    initAngle:

    """

    def __init__(self, gpioNumber=18, initAngle=0, isDebug=False):
        #GPIO4を制御パルスの出力に設定
        self.gpioNumber = gpioNumber
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpioNumber, GPIO.OUT)

        #GPIO.PWM( [ピン番号] , [周波数Hz] )
        flag = True
        self.servo = GPIO.PWM(self.gpioNumber, 300) 
        self.servo.start(20)
        while (True):
            if (flag):
                flag = False
                self.servo.ChangeFrequency(300)
                self.servo.ChangeDutyCycle(20)
            else:
                flag = True
                self.servo.ChangeFrequency(90)
                self.servo.start(20)
            time.sleep(1)

    def initAngle(self, angle):
        self.servo.ChangeDutyCycle(angle)
        time.sleep(fullTime)
        self.servo.ChangeDutyCycle(0)
        self.saveAngle(angle)

    def stop(self):
        self.servo.stop()

    def debug(b):
        self.isDebug = b

    def cleanup(self):
        GPIO.cleanup()

    def saveAngle(self, angle):
        """角度を更新する"""
        self.angle = angle

    def getPreAngle(self):
        """今の角度を取得する"""
        return self.angle

    def changeAngle(self, angle):
        a = angle / 180
        _0 = minPulse
        _180 = maxPulse
        b = _180 - _0
        c = (a * b) + _0 #制御パルス(%)
        #print("minPulse=", minPulse, "maxPulse=", maxPulse)

        deffAngle = math.fabs(angle - self.getPreAngle())
        print("Servo#changeAngle angle=", angle, "Servo#getPreAngle()=", self.getPreAngle())
        waitTime = baseTime + (variTime * (deffAngle/180))
        self.saveAngle(angle)
        self.servoDutyCycle(c, waitTime)

    def addAngle(self, value):
        newAngle = baseTime + self.getPreAngle() + value
        if (180 < newAngle or newAngle < 0):
            print("限界角度です")
            return False
        self.changeAngle(newAngle)
        return True

    def getPWMCycle(self, ms):
        return np.roots([-ms, 1000])

    def servoDutyCycle(self, value, waitTime):
        print("Servo#servoDutyCycle value=", value, "waitTime=", waitTime)
        self.servo.ChangeDutyCycle(value)
        time.sleep(waitTime)
        self.servo.ChangeDutyCycle(0)
        #time.sleep(0.007)

def main():
    #GPIO4を制御パルスの出力に設定
    self.gpioNumber = gpioNumber
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.gpioNumber, GPIO.OUT)

    #GPIO.PWM( [ピン番号] , [周波数Hz] )
    flag = True
    self.servo = GPIO.PWM(self.gpioNumber, 300) 
    self.servo.start(20)
    while (True):
        if (flag):
            flag = False
            self.servo.ChangeFrequency(300)
            self.servo.ChangeDutyCycle(20)
        else:
            flag = True
            self.servo.ChangeFrequency(90)
            self.servo.start(20)
        time.sleep(1)

if __name__ == "__main__":
    main()
