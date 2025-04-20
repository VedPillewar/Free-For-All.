# Free-For-All.
This are the code which any beginner can use for starting and getting boost in his programming career . 


 [1] LANGUAGE:- "Python"

 PROBLEM STATEMENT:- Find whether the number is Armstrong number or not .

 CODE NO :- 1

          n=int(input("enter any number"))
          sum=o
          number=n
          l=len(Str(n))

          while n>0:
          r=n%10
          P=r**l
          sum=sum+P
          n=n//10

          if number==sum:
          print("it is armstrong number")
          else:
          print("not an armstrong number")



[2] LANGUAGE:- "Python"

PROBLEM STATEMENT:- Convert binary to decimal no.

CODE NO :- 2

          n=int(input("enter any number"))
          sum=o
          number=n
          l=len(Str(n))
          mul=0

          while n>0:
          r=n%10
          P=r*(2**mul)
          mul=mul+1
          sum=sum+P
          n=n//10
          print(sum)



 [3] LANGUAGE:- "Python"

 PROBLEM STATEMENT:- To find the count of each char enter through keyboard can be (upper,lower case & digit).

 CODE NO :- 3

      lower_count=0
      upper_count=0
      digit_count=0
      while true:

      n=Str(input("enter any character")

      if(n=='*'):
      break
      else if(n.islower):
      lower_count=lower_count+1
      else if(n.isupper):
      upper_count=upper_count+1
      else if(n.isdigit):
      digit_count=digit_count+1
      else:
      print('invalid input please try again")

      print("lower case count")
      print(lower_count)

      print("upper case count")
      print(upper_count)

      print("digit count")
      print(digit_count)



  [4] LANGUAGE:- "Python"

  PROBLEM STATEMENT:- To find whether the number is prime number or not.

  CODE NO :- 4

    n= int(input("enter any number"))

    p_count=0
    c_count=0

    while n!=-1:

    if n<2:
    print ("neither prime nor composite")
    else:
    for i in range (2,int (n**0.5)+1):
    if n%i==0:
    c_count=c_count+1
    break
    else:
    print("the number is composite")
    p_count=p_count+1

    n=int(input("enter any number(-1 to exit))):
    print("code exited")
    print("prime count is ", p_count)
    print(" composite count is ",c_count)



    
  [5] LANGUAGE:- "Python"

  PROBLEM STATEMENT:- Enter a string and search for a character in it,display the count of the character for which it appears in this string.

  CODE NO :- 5


    Str=input("enter any string")
    target=input("enter any characters")
    rev_str=""

    Ch_count=0

    for Ch is str:
    if(Ch==target):
    Ch_count=Ch_count+1
    print("the count of traget is",Ch_count)

    for n in str:
    rev_str=n + rev_str
    print("the reserved string is",rev_str)



        
  [6]|[A] LANGUAGE:- "C"

  HARDWARE BOARD:-"Arduino Uno"

  PROBLEM STATEMENT:- C code for blinking of LED light on switch on/off.

  CODE NO :- 6


     # define ledpin 13
     # define sw 7
     int Vol=o;
     Void setup();

     pinMode(ledpin,OUTPUT);
     pinMode(sw,INPUT);
     }

     void loop(){
     Vol=digital Read (sw);
     if(vol==HIGH){
     digitalWrite(ledpin,LOW);}
     else{
     digitalWrite(ledpin,HIGH);}}



   [7]|[B] LANGUAGE:- "C"

   HARDWARE BOARD:-"Arduino Uno"

   PROBLEM STATEMENT:- C program to run IR sensor with LED.

   CODE NO :- 7


    int IRsensor=7;
    int ledpin=13;

    void setup(){
    Serial.begin(115200);
    Serial.println("serial working");
    pinMode(lepin,OUTPUT);
    pinMode(IRsensor,INPUT);
    }
    void loop(){
    int sensorstatus=digitalRead(IRsensor);
    if(sensorstatus==0){
    digitalWrite(ledpin,HIGH);
    Serial.println("motion detected");
    }
    else{
    digitalWrite(ledpin,LOW);
    Serial.println("motion not detected");
    }}



    
   [8]|[C] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for blinking of LED.

   CODE NO :- 8


      import Rpi.GPIO as GPIO
      import time
      LED=18
      
      GPIO.setmode(GPIO.BOARD)
      GPIO.setwarnings(False)
      GPIO.setup(LED,GPIO.PUT)
      GPIO.outside(LED,False)
      
      try;
      while True:
      GPIO.output(LED,True)
      print("LED ON")
      time.sleep(1)
      GPIO.output(LED,False)
      print("LED OFF")
      time sleep(1)
      
      finally:
      GPIO.cleanup()



      
    
   [9]|[D] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for blinking of LED by using IR sensors.

   CODE NO :- 9


      import Rpi.GPIO as GPIO
      import time
      LED=18
      SENSOR=16
      
      GPIO.setmode(GPIO.BOARD)
      GPIO.setwarnings(False)
      GPIO.setup(LED,GPIO.OUT)
      GPIO.setup(SENSOR,GPIO.IN)
      GPIO.output(LED,False)
      
      print("IR INITIALIZE")
      time.sleep(5)
      print("IR READY")
      
      try:
      while True:
      if GPIO.input (SENSOR):
      GPIO.output (LED,True)
      print("DETECTED")
      while GPIO.input(SENSOR)
      time.sleep(0.2)
      else:
      GPIO.output (LED,False)
      
      finally:
      GPIO.cleanup()



   [10]|[E] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for ultrasonic sensors interfacing.

   CODE NO :- 10


      import Rpi.GPIO as GPIO
      import time 
      
      GPIO.setup(GPIO.BOARD)
      TRIG=13
      ECHO=18
      
      GPIO.setup(TRIG,GPIO.OUT)
      GPIO.setup(ECHO,GPIO.IN)
      def.distance():
      GPIO.output(TRIG,GPIO.LOW)
      time.sleep(0.001)
      GPIO.output(TRIG,GPIO.HIGH)
      time.sleep(0.001)
      GPIO.output(TRIG,GPIO.LOW)
      
      while GPIO.input(ECHO)==GPIO LOW;
      pulse_end=time.time()
      pulse_duration=pulse_end_pulse_Stord
      distance=pulse duration*34200/2 return distance
      
      try:
      while True:
      dist=distance()
      print("measured distance=%.if cm" %dist)
      time.sleep(1)
      
      finally:
      GPIO.cleanup()
      
     
      
   [11]|[F] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for Servo motor.

   CODE NO :- 11


      import Rpi.GPIO as GPIO
      import time 

      from time import sleep
      GPIO.setwarning(False)
      GPIO.setmode(GPIO.BOARD)
      GPIO.setup(11,GPIO.OUT)
      p=GPIO.PWM(11,50)
      p.start(0)
      
      try:
      while true:
      p.changeDutyCycle(2.5)
      time.sleep(1)
      p.changeDutyCycle(12.5)
      time.sleep(1)

      except KeyboardInterupt:
      print("\n Exiting program")

      finally
      p.stop()
      GPIO.cleanup()
      
      
      
   [12]|[G] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   ADDITINAL REQUIRMENT:- "ThingSpeak" (WebSite) account & channel.

   PROBLEM STATEMENT:- Code for using ultrasonic sensor with "ThingSpeak".

   CODE NO :- 12
   

    import sys
    import urllib.request
    import time
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO_TRIGGER=18
    GPIO_ECHO =15
    
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
    GPIO.setup(GPIO_ECHO,GPIO.IN)
    
    #Enter your API Key here
    myAPI='YUGUBFBLT7YR2KWD'
    
    #URL where we will send the data
    baseURL='https://api.thingspeak.com/update?api_key=%s' %myAPI
    
    def ultrasonic_sensor():
    #set trigger to high
    GPIO.output(GPIO_TRIGGER,True)
    
    #set trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER,False)
    StartTime=time.time()
    StopTime=time.time()
    #save StartTime
    
    while GPIO.input(GPIO_ECHO) == 0:
    StartTime=time.time()
    
    #save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
    StopTime=time.time()
    
    #time difference between start and arrival
    TimeElapsed= StopTime - StartTime
    
    #multiply with sonic speed(34300cm/s) and divide by 2 go and return
    dist= (TimeElapsed*34300)/2
    print(dist)
    return dist
    
    try:
    while True:
    dist = ultrasonic_sensor()
    if isinstance(dist,float):
    dist = '%.2f' % dist 
    print('distance = ',dist)
    
    #sending data to thinkspeak
    conn=urllib.request.urlopen(baseURL+'&field1=%s' % (dist))
    print(conn.read())
    conn.close()
    time.sleep(15)
    else:
    print('error')
    
    finally:
    print("Measurement stopped by User")
    GPIO.cleanup()
	


   [13]|[H] LANGUAGE:- "C"

   HARDWARE BOARD:-"ESP32"

   PROBLEM STATEMENT:- Code for a sensor-based system using ESP32 platform to alert a person if someone touches the locker (Buzzer & Touch sensor).

   CODE NO :- 13


    #define SENSOR 15
    #define BUZZER 13

    void setup() {
    pinMode(SENSOR, INPUT);
    pinMode(BUZZER, OUTPUT);
    Serial.begin(115200);
    }

    void loop() {
    int sensorValue = digitalRead(SENSOR);
    
    if (sensorValue == HIGH) {
    digitalWrite(BUZZER, HIGH);
    Serial.println("Locker Touched!");
    delay(1000); }
    else {
    digitalWrite(BUZZER, LOW);
    Serial.println("Locker Not Touched!");
    delay(1000);
    }
    }

      

   [14]|[I] LANGUAGE:- "C"

   HARDWARE BOARD:-"ESP32"

   PROBLEM STATEMENT:- Code for a DHT11 sensor.

   CODE NO :- 14    


    #include <DHT.h>
    #define DHT11_PIN 14
    DHT dht11(DHT11_PIN, DHT11);

    void setup() {
    Serial.begin(9600);
    dht11.begin();
    }

    void loop() {
    float humi = dht11.readHumidity();
    float tempC = dht11.readTemperature();
    float tempF = dht11.readTemperature(true);
    delay(2000);

    }

    if (isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT11 sensor!");
    } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");
    Serial.print(" | ");
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C ~ ");
    Serial.print(tempF);
    Serial.println("°F");
    }

    delay(2000);
    }

    

  [15]|[J] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for a running servo motor when ever any object comes infornt of camera.

   CODE NO :- 15   


     import cv2
     import RPi.GPIO as GPIO
     import time
 
     SERVO_PIN = 18
     GPIO.setmode(GPIO.BCM)
     GPIO.setup(SERVO_PIN, GPIO.OUT)

     pwm = GPIO.PWM(SERVO_PIN, 50)
     pwm.start(0)


     def set_angle(angle):
     duty = angle / 18 + 2
     GPIO.output(SERVO_PIN, True)
     pwm.ChangeDutyCycle(duty)
     time.sleep(0.5)
     GPIO.output(SERVO_PIN, False)
     pwm.ChangeDutyCycle(0)

     face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

     cap = cv2.VideoCapture(0)

     try:
     while True:
     ret, frame = cap.read()
     if not ret:
     print("Failed to grab frame")
     break

     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     faces = face_cascade.detectMultiScale(gray, 1.3, 5)

     if len(faces) > 0:
     print("Person detected!")
     set_angle(90)  # Rotate servo to 90 degrees
     else: 
     set_angle(0)  # Reset position

     cv2.imshow("Camera Feed", frame)
     if cv2.waitKey(1) & 0xFF == ord('q'):
     break

     finally:
     cap.release()
     cv2.destroyAllWindows()
     pwm.stop()
     GPIO.cleanup()

   

   [16]|[K] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for a running stepper motor when any person come infront of ultrasonic sensor.

   CODE NO :- 16   

     import RPi.GPIO as GPIO
     import time
     
     GPIO.setmode(GPIO.BCM)

     TRIG = 23
     ECHO = 24

     IN1 = 17
     IN2 = 18
     IN3 = 27
     IN4 = 22

     motor_pins = [IN1, IN2, IN3, IN4]

     GPIO.setup(TRIG, GPIO.OUT)
     GPIO.setup(ECHO, GPIO.IN)
     for pin in motor_pins:
          GPIO.setup(pin, GPIO.OUT)
          GPIO.output(pin, False)

     step_seq = [
          [1,0,0,1],
          [1,0,0,0],
          [1,1,0,0],
          [0,1,0,0],
          [0,1,1,0],
          [0,0,1,0],
          [0,0,1,1],
          [0,0,0,1]
     ]

     def get_distance():
          GPIO.output(TRIG, True)
          time.sleep(0.00001)
          GPIO.output(TRIG, False)

          start_time = time.time()
          stop_time = time.time()

          while GPIO.input(ECHO) == 0:
               start_time = time.time()

          while GPIO.input(ECHO) == 1:
               stop_time = time.time()

          elapsed = stop_time - start_time
          distance = (elapsed * 34300) / 2
          return distance

     def move_stepper(steps=512, delay=0.002):
          for _ in range(steps):
               for step in step_seq:
                    for pin in range(4):
                         GPIO.output(motor_pins[pin], step[pin])
                    time.sleep(delay)

     try:
          print("System Ready. Press Ctrl+C to stop.")
          while True:
               dist = get_distance()
               print(f"Distance: {dist:.1f} cm")

               if dist > 2 and dist <= 15:
                    print("Object detected! Rotating stepper motor...")
                    move_stepper()  # One full revolution (512 steps)
                    time.sleep(1)

               time.sleep(0.2)

     except KeyboardInterrupt:
          print("\nCleaning up GPIO and exiting.")
          GPIO.cleanup()



   [17]|[L] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for a running DHT11 sensor and send information on display.

   CODE NO :- 17  


     import Adafruit_DHT
    from RPLCD.i2c import CharLCD
    from time import sleep

    # Sensor setup
    sensor = Adafruit_DHT.DHT11
    dht_pin = 4  # GPIO where DATA of DHT11 is connected

       # LCD setup (check your I2C address, commonly 0x27 or 0x3F)
    lcd = CharLCD('PCF8574', 0x27)

    def read_dht11():
    humidity, temperature = Adafruit_DHT.read_retry(sensor, dht_pin)
    return humidity, temperature

    try:
    while True:
        humidity, temperature = read_dht11()
        if humidity is not None and temperature is not None:
            lcd.clear()
            lcd.write_string(f"Temp: {temperature}C")
            lcd.crlf()
            lcd.write_string(f"Humidity: {humidity}%")
            print(f"Temp: {temperature}C  Humidity: {humidity}%")
        else:
            lcd.clear()
            lcd.write_string("Sensor Error")
            print("Failed to read from DHT11")
        sleep(2)

    except KeyboardInterrupt:
    print("Program stopped by user")
    lcd.clear()



   [18]|[M] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:- Code for jetson nano for do 3d scan and termo vision using camera.

   NECESSARY LIBRARIES:- sudo apt-get update
                         sudo apt-get install -y libusb-1.0-0-dev,
                         sudo apt-get install -y libssl-dev libudev-dev,
                         # Download and install RealSense SDK,
                         sudo apt-get install -y librealsense2-dev,
                         pip3 install pyrealsense2,
                         sudo apt-get install -y python3-opencv,
                         pip3 install pylepton,

   CODE NO :- 18 (A) Code for 3D Scanning using Intel RealSense

      import pyrealsense2 as rs
      import numpy as np
      import cv2

     # Configure the RealSense camera
     pipeline = rs.pipeline()
     config = rs.config()
     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the pipeline
    pipeline.start(config)

    try:
    while True:
        # Wait for frames from the camera
        frames = pipeline.wait_for_frames()
        
        # Get the depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply color map to depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack color and depth images horizontally
        combined_image = np.hstack((color_image, depth_colormap))

        # Display the combined image
        cv2.imshow('3D Scan', combined_image)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except KeyboardInterrupt:
    print("Exiting...")
    pipeline.stop()
    cv2.destroyAllWindows()


(B) Code for Thermal Vision using FLIR Lepton


        import cv2
        import numpy as np
        import pylepton

        # Initialize the FLIR Lepton camera
     with pylepton.lepton() as camera:
    while True:
        # Capture thermal data from the Lepton sensor
        frame, _ = camera.capture()

        # Normalize the thermal image data (for visualization purposes)
        frame_normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)

        # Apply color map to the thermal data
        frame_colored = cv2.applyColorMap(np.uint8(frame_normalized), cv2.COLORMAP_JET)

        # Show the thermal image
        cv2.imshow('Thermal Vision', frame_colored)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


(C) Combined 3D Scanning and Thermal Vision


    import pyrealsense2 as rs
    import numpy as np
    import cv2
    import pylepton
    from time import sleep

    # Configure the RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the RealSense pipeline
    pipeline.start(config)

    # Initialize the FLIR Lepton camera
    thermal_camera = pylepton.lepton()

    try:
    while True:
        # Wait for frames from the RealSense camera
        frames = pipeline.wait_for_frames()

        # Get depth and color frames from RealSense
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Get thermal frame from FLIR Lepton
        thermal_frame, _ = thermal_camera.capture()
        thermal_normalized = cv2.normalize(thermal_frame, None, 0, 255, cv2.NORM_MINMAX)
        thermal_colored = cv2.applyColorMap(np.uint8(thermal_normalized), cv2.COLORMAP_JET)

        # Apply color map to depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Combine RGB, Depth, and Thermal images horizontally
        combined_image = np.hstack((color_image, depth_colormap, thermal_colored))

        # Display the combined result
        cv2.imshow('3D Scan and Thermal Vision', combined_image)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except KeyboardInterrupt:
    print("Exiting...")
    pipeline.stop()
    cv2.destroyAllWindows()
    thermal_camera.close()

     
     
   [19]|[N] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:-Code for using gas senor and when ever any harmfull gas get detect buzzer will start buzz.

   NECESSARY LIBRARIES:- sudo apt update,sudo apt install python3-gpiozero

   CODE NO :- 19


    from gpiozero import DigitalInputDevice, Buzzer
     from time import sleep

    # GPIO pin setup
    gas_sensor = DigitalInputDevice(17)  
    buzzer = Buzzer(18)               

    print("Gas detection system initialized.")

    try:
    while True:
        if gas_sensor.value == 0:
            # 0 = Gas detected (depends on module logic level)
            print("⚠️ Harmful gas detected! Buzzer ON")
            buzzer.on()
        else:
            print("✅ Air is clean. Buzzer OFF")
            buzzer.off()
        sleep(1)

    except KeyboardInterrupt:
    print("Program terminated.")
    buzzer.off()



   [20]|[O] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:-Code for Soil Moisture Sensor with Raspberry Pi.

   NECESSARY LIBRARIES:- sudo apt update,sudo apt install python3-gpiozero

   CODE NO :- 20


     from gpiozero import DigitalInputDevice
    from time import sleep

    # Moisture sensor setup (connected to GPIO17)
    moisture_sensor = DigitalInputDevice(17)

    print("Soil Moisture Monitoring Started.")

    try:
    while True:
        if moisture_sensor.value == 0:
            print("💧 Soil is DRY. Water the plant!")
        else:
            print("🌿 Soil is MOIST. No need to water.")
        sleep(2)

    except KeyboardInterrupt:
    print("Program stopped.")

   
    
   [21]|[P] LANGUAGE:- "Python"

   HARDWARE BOARD:-"Raspberry Pi 4/5"

   PROBLEM STATEMENT:-Code for Soil Moisture Sensor with Buzzer Alert and Watering Servo Motor .

   NECESSARY LIBRARIES:- sudo apt update,sudo apt install python3-gpiozero python3-rpi.gpio

   CODE NO :- 21


    from gpiozero import DigitalInputDevice, Buzzer
    from gpiozero import Servo
    from time import sleep
    from signal import pause

    # Moisture sensor connected to GPIO17
    moisture_sensor = DigitalInputDevice(17)

    # Buzzer on GPIO18
    buzzer = Buzzer(18)

    # Servo on GPIO23
    servo = Servo(23)

    def water_the_plant():
    print("🌊 Activating servo to water the plant...")
    servo.min()  # tilt or open
    sleep(2)
    servo.max()  # return to original position
    sleep(1)
    servo.detach()  # stop signal to avoid jitter

    print("🌿 Soil Monitoring System with Water Drop Started...")

    try:
    while True:
        if moisture_sensor.value == 0:
            print("⚠️ Soil is DRY!")
            buzzer.on()
            water_the_plant()
        else:
            print("✅ Soil is moist.")
            buzzer.off()
        sleep(5)

    except KeyboardInterrupt:
    print("System stopped by user.")
    buzzer.off()
    servo.detach()

