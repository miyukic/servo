# coding: UTF-8
import RPi.GPIO as GPIO
import time
import numpy as np
import math


#「GPIO4出力」でPWMインスタンスを作成する。
#GPIO.PWM( [ピン番号] , [周波数Hz] )
#SG92RはPWMサイクル:20ms(=50Hz), 制御パルス:0.5ms〜2.4ms, (=2.5%〜12%)。
#GPIONumberはGPIO.BCMで指定
fullTime = 0.4
PWMCyclems = 20
minPulse = (0.5 / PWMCyclems) * 100 # -> 2.5%
maxPulse = (2.4 / PWMCyclems) * 100 # -> 12% 
class Servo():

    def __init__(self, gpioNumber=4, initAngle=0):
        #GPIO4を制御パルスの出力に設定
        self.gpioNumber = gpioNumber
        GPIO.setup(self.gpioNumber, GPIO.OUT)
        self.servo = GPIO.PWM(self.gpioNumber, self.getPWMCycle(PWMCyclems))
        self.servo.start(0)
        self.initAngle(initAngle)

    def initAngle(self, angle):
        self.servo.ChangeDutyCycle(angle)
        time.sleep(fullTime)
        self.servo.ChangeDutyCycle(0)
        self.saveAngle(angle)

    def stop(self):
        self.servo.stop()

    def cleanup(self):
        GPIO.cleanup()

    def saveAngle(self, angle):
        """角度を更新する"""
        self.angle = angle

    def getPreAngle(self):
        """今の角度を取得する"""
        return self.angle

    def changeAngle(self, value):
        a = value / 180
        _0 = minPulse
        _180 = maxPulse
        b = _180 - _0
        c = a * b
        #print("minPulse=", minPulse, "maxPulse=", maxPulse)

        deffAngle = math.fabs(value - self.getPreAngle())
        print("Servo#changeAngle value=", value, "Servo#getPreAngle()=", self.getPreAngle())
        waitTime = fullTime * (deffAngle/180)
        self.saveAngle(value)
        self.servoDutyCycle(c, waitTime)

    def getPWMCycle(self, ms):
        return np.roots([-ms, 1000])

    def servoDutyCycle(self, value, waitTime):
        print("Servo#servoDutyCycle value=", value, "waitTime=", waitTime)
        self.servo.ChangeDutyCycle(value)
        time.sleep(waitTime)
        self.servo.ChangeDutyCycle(0)
        #time.sleep(0.007)

#servo.start(0)
#servo.ChangeDutyCycle(2.5)
#time.sleep(0.8)


#servoDutyCycle(2.5)
#servo.stop()
#GPIO.cleanup()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    s = Servo(gpioNumber=4)
    s.changeAngle(0)
    s.changeAngle(180)
    s.cleanup()


