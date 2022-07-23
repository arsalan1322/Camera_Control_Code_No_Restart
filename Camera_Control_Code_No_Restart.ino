/*
    Camera control code with input of cameraPosition
*/

#define pot1 A0 // number indicates motor -> later on replace with Iris, Zoom or Focus
#define fwd_dir 3
#define rev_dir 4
#define fwd_dir_focus 9
#define rev_dir_focus 10


float cameraPosition = 0; // store cameraPosition of motor
long prevTime = 0;
float previousError = 0;
float errorIntegral = 0;
//float resistance = 0;
int globalResistance = 800;
//int delay_time = 1950;
//int delay_time_focus = 2750;
String oldzoom;
String zoom;
//String oldfocus;
//String focus; 

void setup() {
  Serial.begin(9600);
  //pinMode(pot1, INPUT);
  pinMode(fwd_dir, OUTPUT);
  pinMode(rev_dir, OUTPUT);
  pinMode(fwd_dir_focus, OUTPUT);
  pinMode(rev_dir_focus, OUTPUT);
  //attachInterrupt(pot1, readPotentiometer, CHANGE);
  //Serial.print("target voltage");
}

void loop() {
//    int voltage = analogRead(pot1);
//   //Serial.print("resistance pot =");
//   Serial.println(voltage);
//   delay(1);
   
    while (Serial.available()){

      oldzoom = zoom;
      zoom = Serial.readStringUntil('\n');
      Serial.print(zoom);
      Serial.print('\n');

      if (zoom.toInt() < oldzoom.toInt())
      {
        changeMotor(-1, fwd_dir, rev_dir);
     delay(oldzoom.toInt() - zoom.toInt());
     changeMotor(0, fwd_dir, rev_dir);
      }

      else if (zoom.toInt() > oldzoom.toInt())
      {
        changeMotor(1, fwd_dir, rev_dir);
     delay(zoom.toInt() - oldzoom.toInt());
     changeMotor(0, fwd_dir, rev_dir);
      }

      else
      {
        changeMotor(1, fwd_dir, rev_dir);
     delay(zoom.toInt());
     changeMotor(0, fwd_dir, rev_dir);
      }
      
    }
    
    
//    changeMotor(1, fwd_dir, rev_dir);
//     delay(delay_time);
//     changeMotor(0, fwd_dir, rev_dir);
////     changeMotor(-1, fwd_dir, rev_dir);
////      delay(delay_time);
//      
//      
//      changeMotor(1, fwd_dir_focus, rev_dir_focus);
//      delay(delay_time_focus);
//      changeMotor(0, fwd_dir_focus, rev_dir_focus);
//      exit(0);
////      changeMotor(-1, fwd_dir_focus, rev_dir_focus);
////      delay(delay_time_focus);

      
 

 // convert resistance into position of motor
//  if(voltage > globalResistance + 100){ // if resistance increases
//    //cameraPosition++; // clock wise rotation
//    
//    //while(resistance > globalResistance){
//      changeMotor(1, fwd_dir, rev_dir);
//      //resistance -= 5;
//
//  } 
//  else if(voltage < globalResistance - 100){ // if resistance decreases
//    //cameraPosition--; // anti-clock wise rotation
//    
//    //while(resistance < globalResistance){
//      changeMotor(-1, fwd_dir, rev_dir);
//      //resistance += 5;
//    
//  }
//
//  else{
//    changeMotor(0, fwd_dir, rev_dir);
//   //exit(0);
//  }

 
  
  /*globalResistance = resistance; // update global resistance with current resistance
  cameraPosition = globalResistance;
    
    // set target cameraPosition
    float target = 100; // -> later on need to find formula to go from angle to target
                       // Ex. 250 * sin(prevTime/1e6);
    
    // constants to generate control signal to change the motors
    float kp = 1;
    float kd = 0;
    float ki = 0;
    
    // calculate the time difference
    long currentTime = micros();
    //long errorTime = currentTime - prevTime;
    float deltaTime = ((float)(currentTime - prevTime)/1.0e6); // convert to seconds
    prevTime = currentTime;
    
    // store the error and use the error difference to change the motor by supplying the 
    // error margin to the controller to generate the control signal
    float error = cameraPosition - target; // or maybe target - cameraPosition? -> depends on how you connect wires
    
    // calculate rate of change for error (or in simpler terms: derivative) with respect to time
    float derivative = (error - previousError)/(deltaTime);
    
    // calculate the error integral for finite difference approximations
    errorIntegral = errorIntegral + (error * deltaTime);
    
     
    Most Important: Generation of Control Signal
    
    float controlSignal = (kp * error) + (kd * derivative) + (ki * errorIntegral);
    
    // set motor direction
    int rotate;
    if(controlSignal >= 0){
      rotate = 1;
    } else {
      rotate = -1;
    }
    
    // operate the motor
    changeMotor(rotate, fwd_dir, rev_dir);
    
    // store the previous error
    previousError = error;
    */
    //Serial.print(target);
    //Serial.print(" ");
 //   Serial.print(cameraPosition);
 //   Serial.print(" ");
    //Serial.print(error);
    //Serial.print(" ");
  //  Serial.print(resistance);
  //  Serial.println();
    
    
}

void changeMotor(int direction, int forward, int reverse){
  if(direction == 1){
    digitalWrite(forward, HIGH); // rotate clockwise
    digitalWrite(reverse, LOW);
  } 
  else if(direction == -1){
    digitalWrite(forward, LOW); // rotate anticlockwise
    digitalWrite(reverse, HIGH);
  } 
  else {
    digitalWrite(forward, LOW); // do nothing
    digitalWrite(reverse, LOW);
  }
}

void readPotentiometer(){
   int resistance = analogRead(pot1);
   Serial.print("resistance pot =");
   Serial.println(resistance);
  
  // convert resistance into position of motor
  if(resistance > globalResistance){ // if resistance increases
    cameraPosition++; // clock wise rotation
  } 
  else if(resistance < globalResistance){ // if resistance decreases
    cameraPosition--; // anti-clock wise rotation
  }
  
  globalResistance = resistance; // update global resistance with current resistance
  cameraPosition = globalResistance;
}
