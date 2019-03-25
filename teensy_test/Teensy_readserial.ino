void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Serial2.begin(9600);

}

int incomingByte = 0;

char Str1[4] = " ";

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available() > 0) {
                // read the incoming byte:
                String Str1 = Serial2.readString();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(Str1);
        }
        else {Serial.print("No Reading");}

        delay(2000);
        


}
