/*
   File:LASERTANK
   Author: Daniel He
   Date: 2023-10-16
   Description: This program controls a laser turret within the range of motion of half a sphere using servos, 
   a joystick, an OLED screen, a piezo buzzer, and a laser module. The joystick serves as the primary control interface.
   
   Components:
   - Servos for horizontal (myservo) and vertical (yservo) movement
   - Joystick with X (joystickXPin) and Y (joystickYPin) axes
   - Digital pin (joystickbutton) for joystick button press
   - Laser module (LaserPin) for emitting laser beams
   - Piezo buzzer (PiezoPin) for sound effects
   - Ultrasonic Sensor with trigger (pingPin) and echo (echoPin) pins for distance measurement
   - OLED display (U8GLIB_SSD1306_128X64) for visual feedback

   Operation:
   - The program continuously reads joystick input to control the servos for directing the laser.
   - The distance to an object is measured using an Ultrasonic Sensor.
   - Based on the joystick button press, the laser is either held (not fired) or fired, and corresponding visual and audio feedback is provided on the OLED display and piezo buzzer.
   - A smiley face is displayed on the OLED screen, with the size of the head representing the distance to the nearest object.

*/
//include the libraries
#include <U8glib.h> //OLED display library
#include <Wire.h> //I2C communication
#include <Servo.h> //servo motor library

// Create an instance of the U8GLIB library for the OLED display
  U8GLIB_SSD1306_128X64 oled(U8G_I2C_OPT_NO_ACK);

//creates variables for units + connect the devices to the specified pins
Servo myservo; // Horisontal movement servo 
int joystickXPin = A0; //Analog pin for the X axis of the joystick
int joystickYPin = A1; // Analog pin for the y axis of the joystick
int joystickbutton = 3; // digital pin for joystick button press
Servo yservo; //Vertical movement servo
#define LaserPin 8 // digital pin lasermodule
int PiezoPin = 7; //digital pin piezobuzzer
const int pingPin = 2; // Digital Pin for trigger of Ultrasonic Sensor
const int echoPin = 4; // Digital Pin for Echo of Ultrasonic Sensor

bool drawX = false; // Flag to indicate whether to draw the "X"

void setup() {
  myservo.attach(9); // Attach horisontal servo to pin 9
  yservo.attach(12); // Attach vertical servo to pin 12
  Serial.begin(9600); // Initialize the serial monitor communication with baud rate of 9600(for troubleshooting)
  pinMode(LaserPin, OUTPUT);//Set the laser module as an output
  pinMode(joystickbutton, INPUT_PULLUP); // Set the joystickbuttonpin as an input with pull-up resistor
  Wire.begin();  // Initialize I2C communication
  oled.setFont(u8g_font_helvB10); // Set the font of the OLED display
  pinMode(pingPin, OUTPUT); // Set the trigger pin of the ultrasoundssensor as an output
  pinMode(echoPin, INPUT); // Set the echo pin of the ultrasoundssensor as an input
  pinMode(PiezoPin, OUTPUT); // Set the piezo buzzer pin as an output
}

void loop() {
  // Print the current distance measured by the Ultrasonic Sensor for troubleshooting
  Serial.println(getDistance());

  // Read the X and Y values from the joystick
  int joystickXValue = analogRead(joystickXPin);
  int joystickYValue = analogRead(joystickYPin);

  // Map the joystick values to servo angles for horizontal and vertical movement
  int servoAngle = map(joystickXValue, 0, 1023, 0, 180);
  int yservoAngle = map(joystickYValue, 0, 1023, 0, 180);

  // Print joystick values for troubleshooting
  Serial.println("x: " + String(joystickXValue) + "   y: " + String(joystickYValue));//troubleshooting

  // Set the angles for the servo motors based on the joystick values
  yservo.write(yservoAngle);
  myservo.write(servoAngle);
  
  // Print the state of the joystick button for troubleshooting
  Serial.println(joystickbutton);

  // Get and print the distance (did not include since it was from troubleshooting and slowing the code
  /*Serial.print("Distance: "); //helps with troubleshooting
  Serial.print(distance);
  Serial.println(" cm");
  */

// If statement that conditionally control laser firing, update display, and draw elements based on joystick button state
  if (digitalRead(joystickbutton) == HIGH) { // if the joystickbutton is not being pressed
    digitalWrite(LaserPin, LOW); // Laser off
    Serial.println("HOLD"); //Troubleshooting, indicate that the the button is not being pressed
    drawX = false; // Reset the flag so the "X" does not get printed on the OLED display
  } else { //if the joystickbutton is being pressed
    drawX = true; // Set the flag to draw the "X"
    digitalWrite(LaserPin, HIGH); // Laser on
    Serial.println("FIRE");// Trouble shooting,indicate that the button is being pressed
    PiezoSong();// Play the sound on the piezo buzzer
  }

  // Clear the display before drawing
  oled.firstPage();
  do {
    // Draw the crosshair
    drawCrosshair(&oled);

    // Draw the "X" if the flag is set
    if (drawX) {
      drawXInCenter(&oled);
    }

    // Draw the smiley face based on distance
    drawSmiley();
  } while (oled.nextPage());
}

// Function to draw the crosshair
void drawCrosshair(U8GLIB *u8g) {
  int crosshairSize = 20; //  crosshair size
  int centerX = u8g->getWidth() / 2;
  int centerY = u8g->getHeight() / 2;

  // Horizontal line
  u8g->drawHLine(centerX - crosshairSize / 2, centerY, crosshairSize);
  
  // Vertical line
  u8g->drawVLine(centerX, centerY - crosshairSize / 2, crosshairSize);
}

// Function to draw an "X" (crosshair) in the center of the screen using the specified OLED display object
void drawXInCenter(U8GLIB *u8g) {
  
  // Calculate the center coordinates of the OLED display
  int centerX = u8g->getWidth() / 2;
  int centerY = u8g->getHeight() / 2;
  
  // The horisontal line of the crosshair
  u8g->drawLine(centerX - 10, centerY - 10, centerX + 10, centerY + 10);
  
 //the Vertical line of the crosshair
  u8g->drawLine(centerX + 10, centerY - 10, centerX - 10, centerY + 10);
}

// Function to draw the smiley face on the OLED display based on distance measured by the ultrasound sensor
void drawSmiley() {
  // Get the distance from the ultrasound sensor
  float distance = getDistance();
  
  // Map the distance to determine the radius of the smiley face head
  // The head radius is adjusted based on the distance range (20 to 300) to fit within the OLED display
  int headRadius = map(distance, 20, 300, 30, 5);

  // Head  - Draw the head by creating a circle with a radius determined by the mapped distance
  oled.drawCircle(64, 32, headRadius);

  // Eyes -  Draw the eyes at fixed positions with a constant size
  oled.drawDisc(50, 25, 5); // Left eye
  oled.drawDisc(78, 25, 5); // Right eye

  // Mouth - Calculate the radius of the mouth relative to the head size
  int mouthRadius = headRadius - 5; // Adjust the mouth size relative to the head size

  // Draw a series of lines to simulate a smiling mouth
  // Loop through angles from 360 to 180 degrees, decreasing by 5 degrees each iteration
  for (int angle = 360; angle >= 180; angle -= 5) {
  // Calculate the coordinates for the starting point of the line
    float x1 = 64 + mouthRadius * cos(radians(angle));
    float y1 = 32 + mouthRadius * sin(radians(angle));

  // Calculate the coordinates for the ending point of the line (slightly rotated)
    float x2 = 64 + mouthRadius * cos(radians(angle - 5));
    float y2 = 32 + mouthRadius * sin(radians(angle - 5));
    
    // Draw the line segment to create a smiling mouth
    oled.drawLine(x1, y1, x2, y2);
  }
} 
void PiezoSong() {  // Function to play a sound on the piezo buzzer, mimicking a stormtrooper's blaster
  tone(PiezoPin, 1500, 100);// Function to the piezo make a tone of x Hz during x milliseconds
  delay(20); //wait x milliseconds until next sound
  tone(PiezoPin, 1700, 100);
  delay(10); //pause for x milliseconds before the end of the function
}
float getDistance() {
  // read the distance from the ultrasound sensor
  
  // Send a low pulse to the trigger pin
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  
  // Send a high pulse to the trigger pin
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  
  // Stop the pulse by sending a low signal
  digitalWrite(pingPin, LOW);

  // Measure the time it takes for the pulse to return (echo)
  float duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance based on the speed of sound in air
  // The speed of sound is approximately 0.0343 cm/microsecond
  // Distance is divided by 2 since the ultrasound signal travels to the target and back
  float distance = (duration / 2) * 0.0343; 

//return the calculated distance
  return distance;
}
