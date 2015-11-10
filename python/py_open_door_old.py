# -*- coding: utf-8 -*  
import serial  
import time 
import RPi.GPIO as GPIO

recv = "" 

# 打开串口  
ser = serial.Serial("/dev/ttyAMA0", 115200)  
def main(): 
    
    while True: 
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(12, GPIO.HIGH)
        time.sleep(0.2) 
        ser.flushOutput()
        ser.flushInput()
        ser.write("AT+INQ\x0D\x0A")
        print "AT+INQ\x0D\x0A"
        
        # 获得接收缓冲区
        count = 0 
        while count == 0:
            count = ser.inWaiting() 
        if count != 0:  
            # 读取内容并回显  
            recv = ser.read(count)
        #print recv
        t = 0  
        while recv.find(" 0xEC11273800B2") == -1:
            t += 1
            if t > 9:
                break
            time.sleep(0.5)
            count = ser.inWaiting()  
            if count != 0:  
                # 读取内容并回显  
                recv += ser.read(count)
        if t > 9 :
            GPIO.output(12, GPIO.LOW)  
            time.sleep(3)
            continue
        t = 0
        while recv.find("INQE") == -1:
            t += 1
            if t > 9:
                break
            time.sleep(0.5)
            count = ser.inWaiting()  
            if count != 0:  
                # 读取内容并回显  
                recv += ser.read(count)
        if t > 9 :
            GPIO.output(12, GPIO.LOW)  
            time.sleep(3)
            continue
        t = 0
        print recv
        index = recv.find(" 0xEC11273800B2") - 1
        num = recv[index]
        ser.flushInput()
        ser.write("AT+CONN" + num + "\x0D\x0A")
        print "AT+CONN" + num + "\x0D\x0A"
        time.sleep(2.0)
        count = 0
        t = 0
        while count == 0:
            count = ser.inWaiting() 
        recv = 0 

        if count != 0:  
            # 读取内容并回显  
            recv = ser.read(count)
        print recv
        times = 0
        while recv.find("CONNECTED") == -1:
            time.sleep(0.7)
            times += 1
            if times > 3:
                break
            count = ser.inWaiting()  
            if count != 0:  
                # 读取内容并回显  
                recv += ser.read(count)
                #print "CONNECTING:" + recv
        print recv
        time.sleep(0.1) 
        ser.flushInput()
        if times < 4:       
            ser.flushOutput() 
            for i in range(0,5):
                ser.write("\x24\x4D\x3C\x00\x14\x14\x0D\x0A")
                print "\x24\x4D\x3C\x00\x14\x14\x0D\x0A"
                time.sleep(0.1) 
        
 
        # 必要的软件延时  
        time.sleep(0.1)
        if times > 3:
            GPIO.output(12, GPIO.LOW)  
            time.sleep(3)
            continue

        count = 0
        t = 0
        times = 0
        while count == 0:
            time.sleep(0.7)
            times += 1
            if times > 3:
                break
            count = ser.inWaiting() 
        if times > 3:
            GPIO.output(12, GPIO.LOW)  
            time.sleep(3)
            continue 
        recv = 0 
        if count != 0:  
            # 读取内容并回显  
            recv = ser.read(count)
        print recv
        times = 0
        while recv.find("$M>O") == -1:
            time.sleep(0.7)
            times += 1
            if times > 3:
                break
            count = ser.inWaiting()  
            if count != 0:  
                # 读取内容并回显  
                recv += ser.read(count)
                #print "CONNECTING:" + recv
        if times > 3:
            GPIO.output(12, GPIO.LOW)  
            time.sleep(3)
            continue 
        break
    if ser != None:  
        ser.close() 
    GPIO.output(12, GPIO.LOW)  
     
if __name__ == '__main__':  
    try:  
        main()  
    except KeyboardInterrupt:  
        if ser != None:  
            ser.close()
        GPIO.output(12, GPIO.LOW)  