/* 
------------------------------------------------------------------------------------------------------------------------------------------------------------------
In this code, the XBee is setup to communicate using the Serial5 serial interface with the Teensy which shouldn't interfere with any of the sensor inputs 

   Connections:
   XBee:               Teensy:
   5V        -->       Vin
   GND       -->       GND
   DIN       -->       33
   DOUT      -->       34
------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */


#include <XBee.h>
#include <Arduino.h>
//#include <stdlib.h>


//Setup XBee object
XBee xbee = XBee();

//Declare global variables for transfer of multiple values out of the receive function
float   newXCoord, newYCoord, newZCoord;
uint8_t newX[20], newY[20], newZ[20];

//Quadcopter ID# in swarm.  Will be unique to each quadcopter (0.0, 1.0, 2.0 etc.)
//This is initialized as a float to make it easier to append it to the coordinates when sending data
float agentID = 1.0;






/*
-----------------------------------------------------------------------------
  Receive Function
-----------------------------------------------------------------------------
*/
int getNewPos(){

  //Initiate return and data length variables
  int dataAvailable = 0;
  int len;

  //Set up receive object
  Rx16Response rx = Rx16Response();

      //Pull the next data packet from the buffer
      xbee.readPacket();

      //If an error is detected, exit the function
      if (xbee.getResponse().isError()) return 0;
      
      //Ensure that the buffer containes valid data before performing analysis
      if (xbee.getResponse().isAvailable()) {
        //For Monitoring: 
        Serial.println("READING");

        //Set up intermediate array to hold the unsigned character data
        uint8_t rxPayload[100];

            //Assign the response data to the rx object
            xbee.getResponse().getRx16Response(rx);

            //Length of the data in the packet for use in the following for loop
            len = rx.getDataLength();
            
            
            /*DEBUGGING: 
             *Serial.print("Len: ");
             *Serial.println(len);
             */
            

            //WORKAROUND FOR THE FREEZING ISSUE:
            /* After the actual data packet is read, the next packet sometimes contains ~254 bytes of data which the following code can't handle...not sure why or what that data is...
               This could be simply a timing problem since the groundstation test code is constantly sending data but I am not sure how to go about examining the erroneous packet 
               to see what it actually contains.  So, for now the code simply checks that the data packet is not significantly bigger than expected and if it is, it exits the function
               (typical data length is 33-34 bytes).
            */
            if (len>40) return 0;


            //Pull data into intermediate array
            for (int i=0; i<len; i++){
              delayMicroseconds(10);
              rxPayload[i] = rx.getData(i);
            }

      //Split the comma-delimited data array into three separate arrays to hold x,y & z coordinates
      int select = 1;   //Used to select which data array the current value pertains to
      int j=0;          //Used for indexing

      //Update the starting index of the data parsing code to ensure that it starts at the first digit
      //NOTE: I need to double-check that this is necessary... (I don't remember if this is a remnant from troubleshooting the freeze issue or not)  
      //          If it is, will have to add code to allow for negative signs as it will likely cause an issue if the x-coordinate is negative :/ 
      int index=0;
      while(!isDigit((char)rxPayload[index])) index++; 


      //Split the data into 3 character arrays with each one containing a coordinate (data is comma-delimited)
      for (int i=index; i<len; i++){
        //X-value
        if (select ==1){
            //Check that the current value is still a part of the coordinate value
            if (rxPayload[i]==NULL) break;  //Check for EOL
            if (rxPayload[i]==44){          //"44" is the character code for ","
              select++;
              j=0;
            } else {
              newX[j++]=rxPayload[i];       //Copy over if the current value is a part of coordinate value 
            }
        }
        //Y-Value
        else if (select ==2){
            //Check that the current value is still a part of the coordinate value
            if (rxPayload[i]==NULL) break;  //Check for EOL
            if (rxPayload[i]==44){          //"44" is the character code for ","
              select++;
              j=0;
            } else {
              newY[j++]=rxPayload[i];       //Copy over if the current value is a part of coordinate value       
            }
        }
        //Z-Value
        else if (select ==3){
            //Check that the current value is still a part of the coordinate value
            if (rxPayload[i]==NULL) break;  //Check for EOL
            if (rxPayload[i]==44){          //"44" is the character code for ","
              select=1;
              j=0;
            } else {
              newZ[j++]=rxPayload[i];       //Copy over if the current value is a part of coordinate value              
            }          
        }
        
      }

      //Perform conversion on the parsed arrays to turn them into useable numerical values (Using global coords for passing multiple values outside of the function)
      newXCoord = atof((char*)newX);
      newYCoord = atof((char*)newY);
      newZCoord = atof((char*)newZ);  

      //Alert the main loop that new coordinate data is available
      dataAvailable = 1;
      }
      return dataAvailable;  
}








/*
----------------------------------------------------------------------------
  Send Function
----------------------------------------------------------------------------
*/

void sendPos(float xInitCoord, float yInitCoord, float zInitCoord){
  //Length of string representation of the coords (must be set manually but can be larger than actual # of characters)
  int strLen = 11;

  //Number of significant digits following the decimal place (must be set manually, not sure how to make dynamic)
  int decNum = 6;
  
  //Setup arrays to hold intermediate data formats
  /*The length of the txPayload array is the exact length of the final array of characters that represent the coordinates, 
    if this is too long you will get unreadable characters on the receiving end.  Too short and you won't send the whole number.
    However, buffer room can be added by increasing the value of strLen.  This will just add more spaces between your numbers
  */
  uint8_t txPayload[1+(3*strLen)];    //1 byte for the agentID + 3 float coordinates
  char xCoord[220], yCoord[220], zCoord[220], agentNum[220];

  //Convert the float coordinate variables and the agentID# to chars for transfer
  //Format: dtostrf(float value, number of characters, number of decimal places, destination array)
  dtostrf(xInitCoord,strLen,decNum,xCoord);
  dtostrf(yInitCoord,strLen,decNum,yCoord);
  dtostrf(zInitCoord,strLen,decNum,zCoord);
  dtostrf(agentID,1,0,agentNum);

 //Create payload with xyz vals
 //Format: memcpy(array to write to[last index of existing data if applicable], data to add, length of new data)
  memcpy(txPayload,agentNum,1);
  memcpy(&txPayload[1],xCoord,strLen);
  memcpy(&txPayload[1+strLen],yCoord, strLen);
  memcpy(&txPayload[1+(2*strLen)],zCoord,strLen);

  //Generate tx packet
  //Format: Tx16Request(address of receiving XBee, data to transmit, size of data)
  Tx16Request tx = Tx16Request(0x0000, txPayload, sizeof(txPayload));

  //Send data
  xbee.send(tx);
  
}




/*
----------------------------------------------------------------------------
  Code to test the functions
----------------------------------------------------------------------------
*/
void setup() {
  Serial.begin(9600);

  //Set up Serial5 for communicating with the XBee
  Serial5.begin(9600);
  xbee.setSerial(Serial5);

}



void loop() {
delay(100);
  Serial.println("Running...");
  
  int isData = getNewPos();

  if (isData){  
    Serial.print("X: ");
    Serial.println(newXCoord,6);
    Serial.print("Y: ");
    Serial.println(newYCoord,6);
    Serial.print("Z: ");
    Serial.println(newZCoord,6);

  sendPos(newXCoord-10,newYCoord-10,newZCoord-10);
  }
  
}