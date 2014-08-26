/*
  Serial Graphing Sketch
 by Tom Igoe
 Language: Processing
 
 This sketch takes ASCII values from the serial port 
 at 9600 bps and graphs them.
 The values should be comma-delimited, with a newline 
 at the end of every set of values.
 The expected range of the values is between 0 and 1023.
 
 Created 20 April 2005
 Updated 27 June 2008
 */

import processing.serial.*;

int maxNumberOfSensors = 11;       // Arduino has 6 analog inputs, so I chose 6
boolean fontInitialized = false;  // whether the font's been initialized
Serial myPort;                    // The serial port

float[] previousValue = new float[maxNumberOfSensors];  // array of previous values
int xpos = 0;                     // x position of the graph
PFont myFont;                     // font for writing text to the window

char go = 'S';

String[] names = {
  "move", "K1", "K2", "K3", "K4", "K2: aI", "K1: aV", "K4: wP", "K3: wV", "torque", "go"
};
int[] mins = {
  0, 0, 0, 0, 0, -100, -20, 10000, -50, -255, 0
};
int[] maxs = {
  0, 3, 3, 3, 3, 100, 20, -10000, 50, 255, 255
};

void setup () {
  // set up the window to whatever size you want:
  size(1024, 1000);        
  // List all the available serial ports:
  println(Serial.list());
  // I know that the first port in the serial list on my mac
  // is always my  Arduino or Wiring module, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  //String portName = Serial.list()[7];
  //print(portName);
  myPort = new Serial(this, "/dev/tty.usbserial-A600eD2X", 9600);
  myPort.clear();
  // don't generate a serialEvent() until you get a newline (\n) byte:
  myPort.bufferUntil('\n');
  // create a font with the fourth font available to the system:
  myFont = createFont(PFont.list()[3], 14);
  textFont(myFont);
  fontInitialized = true;
  // set inital background:
  background(0);
  // turn on antialiasing:
  smooth();
}

void draw () {
  // nothing happens in the draw loop, 
  // but it's needed to keep the program running
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  print (inString);
  

  // if it's not empty:
  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    
    String[] test = match(inString, "[a-zA-Z]");
    if(null != test)
      return;

    // convert to an array of floats:
    float incomingValues[];
    try {
      incomingValues = float(split(inString, ","));
    } 
    catch (Exception e) {
      return;
    }


    // print out the values
    //  print("length: " + incomingValues.length + " values.\t");
    if (incomingValues.length <= maxNumberOfSensors && incomingValues.length > 0) {
      
      noStroke();
      fill(50);
      rect(380, 0, 300, 60);
      fill(255);
      text(names[0] + ": " + String.valueOf(incomingValues[0]), 400, 20);
      text(names[1] + ": " + String.valueOf(incomingValues[1]), 500, 20);
      text(names[2] + ": " + String.valueOf(incomingValues[2]), 600, 20);
      text(names[3] + ": " + String.valueOf(incomingValues[3]), 500, 50);
      text(names[4] + ": " + String.valueOf(incomingValues[4]), 600, 50);
      
      for (int i = 5; i < incomingValues.length; i++) {

        // map the incoming values (0 to  1023) to an appropriate
        // graphing range (0 to window height/number of values):
        float ypos = map(incomingValues[i], mins[i], maxs[i], 0, height/(incomingValues.length - 5));

        // figure out the y position for this particular graph:
        float graphBottom = (i - 4) * height/(incomingValues.length - 5);
        ypos = ypos + graphBottom;

        // make a black block to erase the previous text:
        noStroke();
        fill(0);
        rect(10, graphBottom+1, 110, 20);

        // print the sensor numbers to the screen:
        fill(255);
        int textPos = int(graphBottom) + 14;
        // sometimes serialEvent() can happen before setup() is done.
        // so you need to make sure the font is initialized before
        // you text():
        if (fontInitialized) {
          text(names[i] + ":" + incomingValues[i] + "    ", 10, textPos);
        }
        // draw a line at the bottom of each graph:
        stroke(127);
        line(0, graphBottom, width, graphBottom);
        // change colors to draw the graph line:
        stroke(64*i, 32*i, 255);
        line(xpos, previousValue[i], xpos+1, ypos);
        // save the current value to be the next time's previous value:
        previousValue[i] = ypos;
      }
    }
    // if you've drawn to the edge of the window, start at the beginning again:
    if (xpos >= width) {
      xpos = 0;
      background(0);
    } 
    else {
      xpos++;
    }
  }
}

void keyPressed() {
  switch (keyCode) {
    case 37:
      println("Left");
      go = 'S';
      break;
    case 39:
      println("Right");
      go = 'S';
      break; 
    case 38:
      println("Up");
      go = 'F';
      break;
    case 40:
      println("Down");
      go = 'B';
      xpos = 0;
      background(0);
      break; 

    default:
      println(keyCode);

      break;
  }
  myPort.write(go);
}

