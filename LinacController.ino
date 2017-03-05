//Serial_Controlled_PS2_Emulator.ino

//MIT License
//
//Copyright (c) 2017 Jeremy Bredfeldt
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

//Description:
//Receives a keystroke over serial port form the PC.
//Sends that key stroke out as a PS2 keyboard stroke
// using digital IO on Arduino.
// Based on the PS2dev library:
// (http://playground.arduino.cc/ComponentLib/Ps2mouse)

//Wiring: 
//  Using an analog MUX for switching the output between
//    the existing PS2 keyboard and Arduino. 
//    See schematic for hardware details.

//  Pins 2 and 3 are outputs to the linac console computer
//    Pin 2 is the data pin, and is wired to A1 on the MUX
//    Pin 3 is the clock pin, and is wire dto B1 on the MUX

//  Pin 4 is used to control the MUX output. When P4 is low,
//    the MUX switches to the PS2 keyboard. When P4 is high,
//    the MUX allows Arduino to control the LINAC.

//  The MUX is the TI cd75hc4052.

#define CLKHALF 20 //half a clock period (usec)
#define CLKFULL 40 //full clock period (usec)
int ps2clk = 3; //clock pin
int ps2data = 2; //data pin
int ctrlpin = 4; //input to MUX used to control the output

void setup() {
  // Initialize the output digital io pins
  pinMode(ps2data, OUTPUT);
  digitalWrite(ps2data, HIGH);
  
  pinMode(ps2clk, OUTPUT);
  digitalWrite(ps2clk, HIGH);

  // Initialize the control pin to enable PS2 keybaord
  pinMode(ctrlpin, OUTPUT);
  digitalWrite(ctrlpin, LOW); //switch to keyboard
  
  //this pin controls the LED on Arduino
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //start the serial port comm with PC
  Serial.begin(9600);
  //Serial.println("Ready");  
}

void gohi (int pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void golo(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void write(unsigned char data)
{
  //create clock and scan code waveforms
  unsigned char i;
  unsigned char parity = 1;
  
  Serial.print("sending ");
  Serial.println(data,HEX);
  
  golo(ps2data);
  delayMicroseconds(CLKHALF);
  golo(ps2clk);	// start bit
  delayMicroseconds(CLKFULL);
  gohi(ps2clk);
  delayMicroseconds(CLKHALF);  
  
  for (i=0; i < 8; i++)
  {
      if (data & 0x01)
	{
	  gohi(ps2data);
	} else {
	golo(ps2data);
      }
      delayMicroseconds(CLKHALF);
      golo(ps2clk);	
      delayMicroseconds(CLKFULL);
      gohi(ps2clk);
      delayMicroseconds(CLKHALF);

      parity = parity ^ (data & 0x01);
      data = data >> 1;
    }
  // parity bit
  if (parity)
    {
      gohi(ps2data);
    } else {
    golo(ps2data);
  }
  delayMicroseconds(CLKHALF);
  golo(ps2clk);	
  delayMicroseconds(CLKFULL);
  gohi(ps2clk);
  delayMicroseconds(CLKHALF);

  // stop bit
  gohi(ps2data);
  delayMicroseconds(CLKHALF);
  golo(ps2clk);	
  delayMicroseconds(CLKFULL);
  gohi(ps2clk);
  delayMicroseconds(CLKHALF);

  delayMicroseconds(50);
  
}

void keypress(unsigned char sc)
{
  //Send the scan code (sc) out over PS2
  //send the key down scan code
  write(sc);
  delay(10);
  //send the key up scan code (always starts with 0xF0)
  write(0xF0);
  delay(10);
  write(sc);  
}

void sendPS2(char* cmd)
{
  //decide which scan code to send
  //need to add more here

  if (strcmp(cmd,"a") == 0) {
    keypress(0x1C); return; 
  } else if (strcmp(cmd,"b") == 0) {
    keypress(0x32); return;
  } else if (strcmp(cmd,"c") == 0) {
    keypress(0x21); return;
  } else if (strcmp(cmd,"d") == 0) {
    keypress(0x23); return;
  } else if (strcmp(cmd,"e") == 0) {
    keypress(0x24); return;  
  } else if (strcmp(cmd,"f") == 0) {
    keypress(0x2B); return;   
  } else if (strcmp(cmd,"g") == 0) {
    keypress(0x34); return;         
  } else if (strcmp(cmd,"h") == 0) {
    keypress(0x33); return;
  } else if (strcmp(cmd,"i") == 0) {
    keypress(0x43); return;
  } else if (strcmp(cmd,"j") == 0) {
    keypress(0x3B); return;
  } else if (strcmp(cmd,"k") == 0) {
    keypress(0x42); return;
  } else if (strcmp(cmd,"l") == 0) {
    keypress(0x4B); return;
  } else if (strcmp(cmd,"m") == 0) {
    keypress(0x3A); return;
  } else if (strcmp(cmd,"n") == 0) {
    keypress(0x31); return;
  } else if (strcmp(cmd,"o") == 0) {
    keypress(0x44); return;
  } else if (strcmp(cmd,"p") == 0) {
    keypress(0x4D); return;
  } else if (strcmp(cmd,"q") == 0) {
    keypress(0x15); return;
  } else if (strcmp(cmd,"r") == 0) {
    keypress(0x2D); return;
  } else if (strcmp(cmd,"s") == 0) {
    keypress(0x1B); return;
  } else if (strcmp(cmd,"t") == 0) {
    keypress(0x2C); return;
  } else if (strcmp(cmd,"u") == 0) {
    keypress(0x3C); return;
  } else if (strcmp(cmd,"v") == 0) {
    keypress(0x2A); return;
  } else if (strcmp(cmd,"w") == 0) {
    keypress(0x1D); return;
  } else if (strcmp(cmd,"x") == 0) {
    keypress(0x22); return;
  } else if (strcmp(cmd,"y") == 0) {
    keypress(0x35); return;
  } else if (strcmp(cmd,"z") == 0) {
    keypress(0x1A); return;
  } else if (strcmp(cmd,"0") == 0) {
    keypress(0x45); return;
  } else if (strcmp(cmd,"1") == 0) {
    keypress(0x16); return;
  } else if (strcmp(cmd,"2") == 0) {
    keypress(0x1E); return;
  } else if (strcmp(cmd,"3") == 0) {
    keypress(0x26); return;
  } else if (strcmp(cmd,"4") == 0) {
    keypress(0x25); return;                    
  } else if (strcmp(cmd,"5") == 0) {
    keypress(0x2E); return;
  } else if (strcmp(cmd,"6") == 0) {
    keypress(0x36); return;
  } else if (strcmp(cmd,"7") == 0) {
    keypress(0x3D); return;
  } else if (strcmp(cmd,"8") == 0) {
    keypress(0x3E); return;
  } else if (strcmp(cmd,"9") == 0) {
    keypress(0x46); return;
  } else if (strcmp(cmd,"enter") == 0) {
    keypress(0x5A); return;    
  } else if (strcmp(cmd,"esc") == 0) {
    keypress(0x76); return;      
  } else if (strcmp(cmd,"F2") == 0) {
    keypress(0x06); return;
  } else if (strcmp(cmd,"up") == 0) {
    write(0xE0); write(0x75); write(0xE0); write(0xF0); write(0x75); return;
  } else if (strcmp(cmd,"left") == 0) {
    write(0xE0); write(0x6B); write(0xE0); write(0xF0); write(0x6B); return;
  } else if (strcmp(cmd,"down") == 0) {
    write(0xE0); write(0x72); write(0xE0); write(0xF0); write(0x72); return;
  } else if (strcmp(cmd,"right") == 0) {
    write(0xE0); write(0x74); write(0xE0); write(0xF0); write(0x74); return; 
  } else if (strcmp(cmd,"home") == 0) {
    write(0xE0); write(0xC6); write(0xE0); write(0xF0); write(0xC6); return;      
  } else if (strcmp(cmd,"end") == 0) {
    write(0xE0); write(0x69); write(0xE0); write(0xF0); write(0x69); return;            
  }
}
  
void loop() {
  
  if (Serial.available() > 0) {
    // Disable PS2 keyboard, enable Arduino control
    digitalWrite(ctrlpin, HIGH);
    
    // read incoming command from serial port, command can be up to 8 bytes
    char cmd[8] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'}; //initialize to all null
    Serial.readBytes(cmd,8);
    delay(100);
	  //echo the serial port command to the PC
    Serial.println(cmd);    
	  //Send the keystroke out on the PS2 bus
    sendPS2(cmd);
    
    // Enable PS2 keyboard, disable Arduino control
    digitalWrite(ctrlpin, LOW);       
  }
  
  delay(10); //delay for 10 msec
}
