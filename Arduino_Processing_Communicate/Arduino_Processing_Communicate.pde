/*
  We extended the original FreeIMU processing code to include a 3D wireframe, created a more detailed depiction
 of the IMU's orientation. The 3D version can be activated by pressing TAB. Pressing 'i' will invert the color
 scheme, wich can make 3D viewing easier. Remember to set the user inputs located at the top of the code: the 
 correct COM port and the initial color scheme. Pressing 's' will show 3D views of all the models.
 
 The rotations are inverted due to the differences between the reference frame of Processing and the reference
 frame of the world. Due to the LH orientation of Processing and the particular RH frame chosen for the world,
 rotations about the X(world) are negative rotations about the Z(Processing), rotations about the Y(world) are
 negative rotations about the X(Processing), and rotations about the Z(world) are negative rotations about the 
 Y(Processing).
 */



// user inputs
//String com = "COM23";
//String com = "COM3";
//String com = "COM22";
String com = "COM3";
int backgroundcolor = #ffffff;
int textcolor = #000000;
// degrees of rotation for the 3D effect, doesn't need to be changed
float deg = 3.0;


String val;     // Data received from the serial port

import processing.serial.*;
Serial myPort;  // Create object from Serial class

float [] Q = new float [4];
float [] theta = new float [4];
float [] Q_alt = new float [4];
float [] Q_gyro = new float [4];
float [] Q_varesano = new float [4];
float [] wc = new float [2];
float [] Euler = new float [3]; // psi, theta, phi
float [] Euler_alt = new float [3];
float [] Euler_gyro = new float [3];
float [] Euler_varesano = new float [3];

float [] Q_stor = new float [4];
float [] Q_stor2 = new float [4];
boolean isRecording = false;
float angleRotated;



PFont font;
boolean _3D_On = false;
boolean altView = false;
boolean splitview = false;
int VIEW_SIZE_X = 1400, VIEW_SIZE_Y = 800;

int xQ = VIEW_SIZE_X/2, xQ_alt = xQ, xQ_gyro = xQ, xQ_varesano = xQ;
int yQ = 350, yQ_alt = 350, yQ_gyro = 350, yQ_varesano = 350;
;
int xQtext = xQ, xQ_alt_text = xQ+200, xQ_varesano_text = xQ-200, xQ_gyro_text = xQ-400;
int yQtext = 50, yQ_alt_text = 50, yQ_gyro_text = 50, yQ_varesano_text = 50;


void setup() 
{
  size(1400, 800, P3D);
  //size(VIEW_SIZE_X, VIEW_SIZE_Y, P3D);
/*myPort = new Serial(this, com, 115200);*/

  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw"); 
  
    // I know that the first port in the serial list on my mac
  // is Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String com = "COM3";
  myPort = new Serial(this, com, 115200);

  delay(100);
  myPort.clear();
  myPort.write("start");
}

// decodeFloat translates the string received from the serial into a float. Changed to use float() instead
float decodeFloat(String inString) {
  //byte [] inData = new byte[4];
  //print("inString:");
  //print(inString);
  float outData = float(inString);
  //print("inString 1:");
  //println(inString.substring(0,4));
  //print("inString 2:");
  //println(inString.substring(4,8));
  //print("inString 3:");
  //println(inString.substring(8,12));
  //print("inString 4:");
  //println(inString.substring(12,16));
  //inString = inString.substring(2, 10); // discard the leading "f:"
  //inData[0] = (byte) unhex(inString.substring(0, 1));
  //inData[1] = (byte) unhex(inString.substring(1, 2));
  //inData[2] = (byte) unhex(inString.substring(2, 3));
  //inData[3] = (byte) unhex(inString.substring(3, 4));

  //int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  //return Float.intBitsToFloat(intbits);
  return outData;
}



void readQ() {
  
  if (myPort.available() >= 18) {
    String inputString = myPort.readStringUntil((int) '\n');
    //print(inputString);
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      //print(inputStringArr[0]);
      //print(inputStringArr[1]);
      //print(inputStringArr[2]);
      //println(inputStringArr[3]);
      //int stringlengthcheck = inputStringArr.length;
      //print(stringlengthcheck);
      if (inputStringArr.length == 4) { // q1,q2,q3,q4,\r\n so we have 5 elements
      print("inputStringArr:");
      println(inputStringArr);
        Q[0] = float(inputStringArr[0]);
        Q[1] = float(inputStringArr[1]);
        Q[2] = float(inputStringArr[2]);
        Q[3] = float(inputStringArr[3]);
        //Q[0] = decodeFloat(inputStringArr[0]);
        //Q[1] = decodeFloat(inputStringArr[1]);
        //Q[2] = decodeFloat(inputStringArr[2]);
        //Q[3] = decodeFloat(inputStringArr[3]);
        //Q_alt[0] = decodeFloat(inputStringArr[4]);
        //Q_alt[1] = decodeFloat(inputStringArr[5]);
        //Q_alt[2] = decodeFloat(inputStringArr[6]);
        //Q_alt[3] = decodeFloat(inputStringArr[7]);
        //Q_gyro[0] = decodeFloat(inputStringArr[8]);
        //Q_gyro[1] = decodeFloat(inputStringArr[9]);
        //Q_gyro[2] = decodeFloat(inputStringArr[10]);
        //Q_gyro[3] = decodeFloat(inputStringArr[11]);
        //Q_varesano[0] = decodeFloat(inputStringArr[12]);
        //Q_varesano[1] = decodeFloat(inputStringArr[13]);
        //Q_varesano[2] = decodeFloat(inputStringArr[14]);
        //Q_varesano[3] = decodeFloat(inputStringArr[15]);
        //wc[0] = decodeFloat(inputStringArr[16]);
        //wc[1] = decodeFloat(inputStringArr[17]);
        print("Q:");
        println(Q);
        //print("Q0 = ");
        //print(Q[0]);
        //print("Q1 = ");
        //println(Q[1]);
        //print("Q2 = ");
        //print(Q[2]);
        //print("Q3 = ");
        //print(Q[3]);
      }
    }
  }
}



void buildBoxShape() {
  // this builds the solid box, or the red wireframe in the 3D case
  if (_3D_On || altView) {
    stroke(255, 100, 80);
    strokeWeight(2);
    noFill();
  } else {
    noStroke();
  }
  beginShape(QUADS);

  //Z+ (to the drawing area)
  if (!_3D_On && !altView) {
    fill(#00ff00);
  }
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  //Z-
  if (!_3D_On && !altView) {
    fill(#0000ff);
  }
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);

  //X-
  if (!_3D_On && !altView) {
    fill(#ff0000);
  }
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);

  //X+
  if (!_3D_On && !altView) {
    fill(#ffff00);
  }
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);

  //Y-
  if (!_3D_On && !altView) {
    fill(#ff00ff);
  }
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);

  //Y+
  if (!_3D_On && !altView) {
    fill(#00ffff);
  }
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  endShape();
}

void buildBoxShape2() {
  // this builds the secondary, blue wireframe for the 3D effect
  if (_3D_On || altView) {
    stroke(10, 50, 255);
    strokeWeight(2);
    noFill();
  } else {
    noStroke();
  }
  beginShape(QUADS);

  //Z+ (to the drawing area)
  if (!_3D_On && !altView) {
    fill(#00ff00);
  }
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  //Z-
  if (!_3D_On && !altView) {
    fill(#0000ff);
  }
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);

  //X-
  if (!_3D_On && !altView) {
    fill(#ff0000);
  }
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);

  //X+
  if (!_3D_On && !altView) {
    fill(#ffff00);
  }
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);

  //Y-
  if (!_3D_On && !altView) {
    fill(#ff00ff);
  }
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);

  //Y+
  if (!_3D_On && !altView) {
    fill(#00ffff);
  }
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  endShape();
}

void buildBoxShape3() {
  // this builds the secondary, blue wireframe for the 3D effect
  if ( altView) {
    stroke(10, 255, 50);
    strokeWeight(2);
    noFill();
  } else {
    noStroke();
  }
  beginShape(QUADS);

  //Z+ (to the drawing area)
  if (!_3D_On && !altView) {
    fill(#00ff00);
  }
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  //Z-
  if (!_3D_On && !altView) {
    fill(#0000ff);
  }
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);

  //X-
  if (!_3D_On && !altView) {
    fill(#ff0000);
  }
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);

  //X+
  if (!_3D_On && !altView) {
    fill(#ffff00);
  }
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);

  //Y-
  if (!_3D_On && !altView) {
    fill(#ff00ff);
  }
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);

  //Y+
  if (!_3D_On && !altView) {
    fill(#00ffff);
  }
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  endShape();
}


void drawCube() {  
  pushMatrix();
  translate(xQ, yQ, 0);
  scale(4.5, 4.5, 4.5);

  // negative angles due to lefthanded coordinate system of Processing

  //rotateZ(Euler[0]);
  //rotateX(-Euler[1]);
  //rotateY(-Euler[2]);
  
  rotateY(Euler[0]);
  rotateZ(-Euler[1]);
  rotateX(-Euler[2]);
  
  buildBoxShape();

  popMatrix();
}

void drawCube2() {  
  // this manipulates the secondary, blue wireframe for the 3D effect
  if (!_3D_On && !altView && !splitview) {
    return;
  } else if (altView || splitview) {
    pushMatrix();//blue
    translate(xQ_alt, yQ_alt, 0);//(x,y,z); x points right, y points down, z point out of screen
    scale(4.5, 4.5, 4.5);     

    rotateZ(-Euler_alt[2]);
    rotateX(-Euler_alt[1]);
    rotateY(-Euler_alt[0]);      
    buildBoxShape2();    
    popMatrix();
  } else {
    pushMatrix();//blue
    translate(300, 350, 0);//(x,y,z); x points right, y points down, z point out of screen
    scale(4.5, 4.5, 4.5);

    // 3D effect acheived by a slight rotation about the Z(world) axis, or Y(processing) axis
    rotateY(deg/180*3.14);
    rotateZ(-Euler_gyro[2]);
    rotateX(-Euler_gyro[1]);
    rotateY(-Euler[0]); 

    buildBoxShape2();

    popMatrix();
  }
}

void drawCube3() {  
  // this manipulates the secondary, blue wireframe for the 3D effect
  if (!_3D_On && !altView && !splitview) {
    return;
  } else if (altView || splitview) {
    pushMatrix();//blue
    translate(xQ_gyro, yQ_gyro, 0);//(x,y,z); x points right, y points down, z point out of screen
    scale(4.5, 4.5, 4.5);     

    rotateZ(-Euler_gyro[2]);
    rotateX(-Euler_gyro[1]);
    rotateY(-Euler_gyro[0]);      
    buildBoxShape3();    
    popMatrix();
  } else {
  }
}
void drawCube4() {  
  // this manipulates the secondary, blue wireframe for the 3D effect
  if (!_3D_On && !altView && !splitview) {
    return;
  } else if (altView || splitview) {
    pushMatrix();//blue
    translate(xQ_varesano, yQ_varesano, 0);//(x,y,z); x points right, y points down, z point out of screen
    scale(4.5, 4.5, 4.5);     

    rotateZ(-Euler_varesano[2]);  // change this and it will be all
    rotateX(-Euler_varesano[1]);
    rotateY(-Euler_varesano[0]);      
    buildBoxShape3();    
    popMatrix();
  } else {
  }
}

void draw() {  
 
  readQ();
  
  //println(Q[1]);
  
  quaternionToEuler(Q, Euler);
  quaternionToEuler(Q_alt, Euler_alt);
  quaternionToEuler(Q_gyro, Euler_gyro);
  quaternionToEuler(Q_varesano, Euler_varesano);

  // user-generated, from top
  background(backgroundcolor);
  fill(textcolor);

  textFont(font, 20);
  text("Q_triad:\n" + Q_alt[0] + "\n" + Q_alt[1] + "\n" + Q_alt[2] + "\n" + Q_alt[3], xQ_alt_text, yQ_alt_text);
  text("Q_gyro:\n" + Q_gyro[0] + "\n" + Q_gyro[1] + "\n" + Q_gyro[2] + "\n" + Q_gyro[3], xQ_gyro_text, yQ_gyro_text);
  text("Q_TVCF:\n" + Q[0] + "\n" + Q[1] + "\n" + Q[2] + "\n" + Q[3]+"\n" + wc[0]+"\n" + wc[1], xQtext, yQtext);
  text("Q_fb:\n" + Q_varesano[0] + "\n" + Q_varesano[1] + "\n" + Q_varesano[2] + "\n" + Q_varesano[3]+"\n", xQ_varesano_text, yQ_varesano_text);
  //text("Euler Angles:\npsi(z)  : " + degrees(Euler[0]) + "\ntheta(y): " + degrees(Euler[1]) + "\nphi (x): " + degrees(Euler[2]), 250, 160);
  //text("Alt angles: \npsi : " + degrees(Euler_alt[0]) + "\ntheta: " + degrees(Euler_alt[1]) + "\nphi  : " + degrees(Euler_alt[2]) + "\nangle : " + degrees(angleRotated), 20, 160);
  text("wireframe: " + altView + "     split view: " + splitview, VIEW_SIZE_X/2-200, 780);
  text("Thanks varesano", VIEW_SIZE_X/2, 800);

  drawCube4();
  drawCube3();
  drawCube2();
  drawCube();

  
}

//3-2-1 Euler Angles, ZYX, RPY angles
// See Sebastian O.H. Madwick report "An efficient orientation filter for inertial and intertial/magnetic sensor arrays
void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
  //print(euler[0]);
  //print(euler[1]);
  //println(euler[2]);
}



// called by Processing whenever a key is pressed, key is a system variable set to the key most recently pressed
void keyPressed() {
  if (key == TAB) {
    _3D_On = !_3D_On;
  }
  if (key == 'i') {
    int temp = backgroundcolor;
    backgroundcolor = textcolor;
    textcolor = temp;
  }
  if (key == 'a') {
    altView = !altView;
  }
  if (key == 'r') {
    recordQuat();
  }

  if (key == 's') {
    if (splitview == true) {
      xQ = VIEW_SIZE_X/2; 
      yQ = 350;
      xQ_alt = xQ; 
      xQ_gyro = xQ;
      yQ_alt_text = 50;
      yQ_varesano_text = 50;


      xQtext = xQ; 
      xQ_alt_text = xQ+200; 
      xQ_varesano_text = xQ-200;
      xQ_gyro_text = xQ-400;
    } else {
      xQ = VIEW_SIZE_X/2-180; 
      xQ_alt = VIEW_SIZE_X/2+220; 
      xQ_gyro = xQ_alt;
      xQ_varesano = xQ;
      yQ = 250;
      yQ_gyro = yQ;
      yQ_alt = 550;
      yQ_alt_text = 400;
      yQ_varesano = yQ_alt;
      yQ_varesano_text = yQ_alt_text;
      //xQ = VIEW_SIZE_X/2; 
      //xQ_alt = VIEW_SIZE_X-300; 
      //xQ_gyro = 300;

      xQtext = xQ-200; 
      xQ_alt_text = xQ_alt+100; 
      xQ_gyro_text = xQ_gyro+100;
      xQ_varesano_text =xQtext;
      ;
    }
    splitview = !splitview;
  }
}

void recordQuat() {
  if (isRecording) {
    Q_stor2[0] = Q[0]; 
    Q_stor2[1] = Q[1]; 
    Q_stor2[2] = Q[2]; 
    Q_stor2[3] = Q[3]; 
    //invert Q_stor2
    Q_stor2[1] *=-1; 
    Q_stor2[2] *=-1; 
    Q_stor2[3] *=-1; 
    float Q_e[] = new float [4];
    Q_e[1] = Q_stor[0]*Q_stor2[1] - Q_stor[3]*Q_stor2[2] + Q_stor[2]*Q_stor2[3] + Q_stor[1]*Q_stor2[0];
    Q_e[2] = Q_stor[0]*Q_stor2[2] + Q_stor[3]*Q_stor2[1] + Q_stor[2]*Q_stor2[0] - Q_stor[1]*Q_stor2[3];
    Q_e[3] = Q_stor[0]*Q_stor2[3] + Q_stor[3]*Q_stor2[0] - Q_stor[2]*Q_stor2[1] + Q_stor[1]*Q_stor2[2];
    Q_e[0] = Q_stor[0]*Q_stor2[0] - Q_stor[3]*Q_stor2[3] - Q_stor[2]*Q_stor2[2] - Q_stor[1]*Q_stor2[1];

    angleRotated = 2*acos(Q_e[0]);

    isRecording = false;
  } else {
    Q_stor[0] = Q[0]; 
    Q_stor[1] = Q[1]; 
    Q_stor[2] = Q[2]; 
    Q_stor[3] = Q[3]; 
    angleRotated = 0;
    isRecording = true;
  }
}
