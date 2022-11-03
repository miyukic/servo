#!/usr/bin/env python3
# coding: UTF-8
import RPi.GPIO as GPIO
import time
import numpy as np
import math

"""
モジュールとして import することも、このプログラム単体で使用することもできます。
if __name__ == "__main__":
の中にServoクラスの使い方の例を書いています。
"""
#======================================================#
#SG90というサーボモーターを基準に作成されています
#https://akizukidenshi.com/catalog/g/gM-08761/
#SG90はPWMサイクル:20ms(=50Hz) 制御パルス:0.5ms〜2.4ms
#トルク:1.8kgf・cm 制御角:±約90°(180°)
#======================================================#
#=====グローバル変数の説明=============================#
#fullTimeはサーボモーターが180度移動するときにかかる時間です
#このプログラムではモーターの移動量によってかかる時間を調整しています
#varTimeは例えば90度分の移動なら半分になりますが
#baseTimeはモーターの移動量によらず常に一定です
#なので移動量が0度であっても、baseTimeは不変です
#【例】移動量が90度の場合は varTime * ( 90 / 180 ) + baseTime 秒になります
#======================================================#
baseTime = 0.080
variTime = 0.330 - baseTime
fullTime = baseTime + variTime #SG90は60度で0.1秒なので180度で0.3秒ですが少し多めにしています
#=====グローバル変数の説明=============================#
#PWM制御パルスの割合が必要なので計算しています
#minPulseTime/maxPulseTime
#======================================================#
#改修
PWMCyclems = 20.0
minPulseTime = 0.5 # ms
maxPulseTime = 2.4 # ms
minPulse = (minPulseTime / PWMCyclems) * 100 # -> 2.5%
maxPulse = (maxPulseTime / PWMCyclems) * 100 # -> 12% 
class Servo():
    """ 
    gpioNumber:GPIO
    initAngle:

    """

    def __init__(self, gpioNumber=4, initAngle=0, isDebug=False):
        #GPIO4を制御パルスの出力に設定
        self.gpioNumber = gpioNumber
        GPIO.setup(self.gpioNumber, GPIO.OUT)

        #GPIO.PWM( [ピン番号] , [周波数Hz] )
        self.servo = GPIO.PWM(self.gpioNumber, self.getPWMCycle(PWMCyclems))
        self.servo.start(0)
        self.initAngle(initAngle)
        self.isDebug = isDebug

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

#servo.start(0)
#servo.ChangeDutyCycle(2.5)
#time.sleep(0.8)


#servoDutyCycle(2.5)
#servo.stop()
#GPIO.cleanup()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    s = Servo(gpioNumber=16)
    s.changeAngle(0)
    time.sleep(1)
    #while (True):
    #    if (s.addAngle(10)): #首爾
    #        time.sleep(0.000)
    #    else:
    #        break

    while (True):
        a = 0
        try:
            print("角度を入力してください:")
            a = int(input())
        except:
            break
        s.changeAngle(a)
    s.cleanup()


