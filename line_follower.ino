// Connect the HC-05 TX to Arduino pin 0 TX. 
// Connect the HC-05 RX to Arduino pin 1 RX
// Connect the HC-05 STATE pin to Arduino pin 4.
#define enA 2
#define enB 3

#define inA1 6 //Định nghĩa chân in1 của động cơ A // A is left tire
#define inA2 7 //Định nghĩa chân in2 của động cơ A
#define inB1 8 //Định nghĩa chân in1 của động cơ B // B is right tire
#define inB2 9 //Định nghĩa chân in2 của động cơ B

#define linesens1 10 //Định nghĩa chân cảm biến line 1
#define linesens2 11 //Định nghĩa chân cảm biến line 2
#define linesens3 12 //Định nghĩa chân cảm biến line 3

#define MAX_SPEED 255
char t;

// BTconnected will = false when not connected and true when connected
boolean BTconnected = false;

// connect the STATE pin to Arduino pin D4
const byte BTpin = 4;

int statusCar = 0; // default 0 

void setup() {
  // set the BTpin for input
    pinMode(BTpin, INPUT);
    
  // set pin for motor and sensor
    pinMode(inA1, OUTPUT);//Set chân in1 của dc A là output
    pinMode(inA2, OUTPUT);//Set chân in2 của dc A là output
    pinMode(inB1, OUTPUT);//Set chân in1 của dc B là output
    pinMode(inB2, OUTPUT);//Set chân in2 của dc B là output
    pinMode(linesens1, INPUT);//Set chân cảm biến 1 là input
    pinMode(linesens2, INPUT);//Set chân cảm biến 2 là input
    pinMode(linesens3, INPUT);//Set chân cảm biến 3 là input

    // Start serial communication with the bluetooth module
    // HC-05 default serial speed for communication mode is 9600 but can be different
    Serial.begin(9600);  

}

void loop() {
    if(BTconnected) // if connect, use bluetooth to control
    {
      if(Serial.available())
        {
          t = Serial.read();
          Serial.println(t);
          bluetoothControl(inB1, inB2, inA1, inA2, t);
        }
      else { BTconnected = false;}; // if disconnect or interupt or lost connection
    }

    // if the HC-05 has made a connection
    if ( digitalRead(BTpin)==HIGH)  { BTconnected = true;} // if have connection

    else // else use line follow function
      darkLineFollower (inB1, inB2, inA1, inA2, linesens1, linesens2, linesens3);
    //delayMicroseconds(1);
}

void bluetoothControl(byte inR1, byte inR2, byte inL1, byte inL2, char sig) {
    if(sig == 'F'){            // Forward
      robotMover( inR1, inR2, inL1, inL2, 1);// tiến thẳng
    }
     
    else if(sig == 'B'){      // Back
      robotMover( inR1, inR2, inL1, inL2, 2);// lùi
    }
     
    else if(sig== 'L'){      // Left
      robotMover( inR1, inR2, inL1, inL2, 5);// rẽ trái
    }
     
    else if(sig == 'R'){      // Right
      robotMover( inR1, inR2, inL1, inL2, 6);// rẽ phải
    }
    
    else if(sig == 'G'){      // Forward Left - without PWM, it's Spin Left
      robotMover( inR1, inR2, inL1, inL2, 3);// stop
    }

    else if(sig == 'I'){      // Forward Right - without PWM, it's Spin Right
      robotMover( inR1, inR2, inL1, inL2, 4);// stop
    }
    
    else if(sig == 'S'){      //STOP (all motors stop)
      robotMover( inR1, inR2, inL1, inL2, 0);// stop
    }
    delay(5);
}

void darkLineFollower (byte inR1, byte inR2, byte inL1, byte inL2, byte sen1, byte sen2, byte sen3)
{
  //Hàm điều khiển robot bám line màu tối
  //inR1, inR2 và inL1, inL2 là các chân tín hiệu lần lượt điều khiển động cơ di chuyển bên phải và trái
  //sen1 đến sen3 là chân nhận tín hiệu từ cảm biến hồng ngoại
  //Bây giờ thì lập trình thôi
  switch (deviationDarkLine3Sensor (sen1, sen2, sen3))
  {
    case 4:
      robotMover( inR1, inR2, inL1, inL2, 0);// stop
      break;
    case -1:
      robotMover( inR1, inR2, inL1, inL2, 6);// rẽ phải
      break;
    case -2:
      robotMover( inR1, inR2, inL1, inL2, 6);
      break;
    case 1:
      robotMover( inR1, inR2, inL1, inL2, 5);// rẽ trái
      break;
    case 2:
      robotMover( inR1, inR2, inL1, inL2, 5);
      break;
    case 0:
      robotMover( inR1, inR2, inL1, inL2, 1);// tiến thẳng
      break;
    case 3:
      robotMover( inR1, inR2, inL1, inL2, 2);// lệch vạch thì lùi
      break;
          
  }
  
}
void robotMover (byte inR1, byte inR2, byte inL1, byte inL2, byte action)
{
  /*
  inR1 inR2 là 2 chân tín hiệu động cơ bên phải
  inL1 inL2 là 2 chân tín hiệu động cơ bên trái
  action= 0 đứng yên
  action =1 đi thẳng
  action =2 lùi lại
  action =3 quay trái
  action =4 quay phải
  action =5 rẽ trái
  action =6 rẽ phải

  */
  switch (action)
  {
    case 0:// không di chuyển
      motorControlNoSpeed(inR1, inR2, 0);
      motorControlNoSpeed(inL1, inL2, 0);
      break;
    case 1://đi thẳng
      motorControlWithSpeed(inR1, inR2, 1, 200, 200);
      motorControlWithSpeed(inL1, inL2, 1, 200, 200);
      break;
    case 2:// lùi lại
      motorControlNoSpeed(inR1, inR2, 2);
      motorControlNoSpeed(inL1, inL2, 2);
      break;
    case 3:// quay trái
      motorControlNoSpeed(inR1, inR2, 1);
      motorControlNoSpeed(inL1, inL2, 2);
      break;
    case 4:// quay phải
      motorControlNoSpeed(inR1, inR2, 2);
      motorControlNoSpeed(inL1, inL2, 1);
      break;
    case 5:// rẽ trái
      motorControlWithSpeed(inR1, inR2, 1, 0, 150);
      motorControlWithSpeed(inL1, inL2, 0, 128, 0); //weak momentum
      break;
    case 6:// rẽ phải
      motorControlWithSpeed(inR1, inR2, 2, 0, 80);
      motorControlWithSpeed(inL1, inL2, 1, 180, 0);
      break;
    default:
      action = 0;
      
  }
}


void motorControlNoSpeed (byte in1, byte in2, byte direct)
{
  // in1 and in2 are 2 signal pins to control the motor
  // en is the enable pin
  // the default speed is the highest
  // direct includes:
  //    0:Stop
  //    1:Move on 1 direct
  //    2:Move on another direct
  switch (direct)
  {
    case 0:// Dừng không quay
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      break;
    case 1:// Quay chiều thứ 1
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, MAX_SPEED);
      analogWrite(enB, MAX_SPEED);
      
      break;
    case 2:// Quay chiều thứ 2
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, MAX_SPEED);
      analogWrite(enB, MAX_SPEED);
      
      break;
      //default:
  }
}

void motorControlWithSpeed (byte in1, byte in2, byte direct, int spdA, int spdB)
{
  // in1 and in2 are 2 signal pins to control the motor
  // en is the enable pin
  // the default speed is the highest
  // direct includes:
  //    0:Stop
  //    1:Move on 1 direct
  //    2:Move on another direct
  switch (direct)
  {
    case 0:// Dừng không quay
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      break;
    case 1:// Quay chiều thứ 1
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      if(spdA != 0) analogWrite(enA, spdA);
      if(spdB != 0) analogWrite(enB, spdB);
      
      break;
    case 2:// Quay chiều thứ 2
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      if(spdA != 0) analogWrite(enA, spdA);
      if(spdB != 0) analogWrite(enB, spdB);
      break;
      
      //default:
  }
}

boolean IFSensor (byte PinNumb)
{
//0 sang
//1 toi
//  Serial.println(digitalRead (PinNumb));
  return(digitalRead (PinNumb));   
}

int deviationDarkLine3Sensor (int PinNumb1, int PinNumb2, int PinNumb3)
{
  
  int left = 0; //biến kiểm tra lệch trái
  int right = 0; // biến kiểm tra lệch phải
  left = IFSensor (PinNumb1)+IFSensor (PinNumb2); //kiểm tra mấy cảm biến trái ở trong màu đen
  right= IFSensor (PinNumb2)+IFSensor (PinNumb3); //kiểm tra mấy cảm biến phải ở trong màu đen
//  Serial.print("p1:");
//  Serial.println(IFSensor (PinNumb1));
//  Serial.print("p2:");
//  Serial.println(IFSensor (PinNumb2));
//  Serial.print("p3:");
//  Serial.println(IFSensor (PinNumb3));

  if ((left!=0) || (right!=0))
  {  
    statusCar = left - right;
    return left - right;
  }
  else
  {  
    if (statusCar == 0)
      return 0 ;
    
    if (statusCar == -1)
    {
       return -1;
    }
     
    if (statusCar == 1)
    {
       return 1;
    }
  }
  /*
  Kết quả trả về là 0 là không lệch
  Âm là lệch trái
  Dương là lệch phải
  */

}
