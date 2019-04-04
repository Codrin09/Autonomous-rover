char serialByte = '0';

void setup()
{
  // communication with the host computer
    Serial.begin(9600);  
 
    Serial.println("Do not power the BT module");
    Serial.println(" ");
    Serial.println("On the BT module, press the button switch (keep pressed, and at the same time power the BT module");
    Serial.println("The LED on the BT module should now flash on/off every 2 seconds");
    Serial.println("Can now release the button switch on the BT module");
    Serial.println(" ");
    Serial.println("After entering AT mode, type 1 and hit send");
    Serial.println(" ");
 
 
    // wait for the user to type "1" in the serial monitor
    while (serialByte !='1')
    {
        if ( Serial.available() )   {  serialByte = Serial.read();  }
    }  
 
 
    // communication with the BT module on serial1
    Serial2.begin(115200);
 
    Serial.println(" ");
    Serial.println("AT mode.");
    Serial.println("Remember to to set Both NL & CR in the serial monitor.");
    Serial.println("The HC-05 accepts commands in both upper case and lower case");
    Serial.println(" "); 

}

void loop() // run over and over
{
  // listen for communication from the BT module and then write it to the serial monitor
  if ( Serial2.available() )   {  Serial.write( Serial2.read() );  }
 
  // listen for user input and send it to the HC-05
  if ( Serial.available() )   {  Serial2.write( Serial.read() );  }
}
