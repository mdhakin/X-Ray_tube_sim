#include <DS3231.h>
#include <Wire.h>
#include <SD.h>

// REAL TIME CLOCK STUFF
DS3231 myRTC;

bool century = false;
bool h12Flag;
bool pmFlag;

// **************

enum tubeState {
  idle,
  preheating,
  shot
};

enum RadShot {
  ts1,
  ts2,
  none
};


RadShot currentShot = none;

tubeState ts = idle;
int iDelay = 20;
// Used to hold data that comes from the serial port
String readString; 

// holds serial port incoming messages
String inputString = "";

// Message from the serial port is ready
bool stringComplete = false; 

String strToWrite = "";

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long period = 1000;  //the value is a number of milliseconds

unsigned long startMillis2;  //some global variables available anywhere in the program
unsigned long currentMillis2;
unsigned long period2 = 5000;  //the value is a number of milliseconds

String FMState = "!FM0";

bool bShootingRays = false;
bool bPower = false;

bool autoSend = false;

/*
bool printAM = false;
bool printPH = false;
bool printER = false;
bool printMA = false;
bool printKV = false;
bool printAT = false;
*/

unsigned int AM0 = 1;
unsigned int AM1 = 1;
unsigned int AM2 = 1;
unsigned int AM3 = 1;
unsigned int AM4 = 45000;
unsigned long AM5 = 150608;

unsigned int AT0 = 27;
unsigned int AT1 = 24;
unsigned int AT2 = 23;
unsigned int AT3 = 19;
unsigned int AT4 = 5777;

unsigned int ph1 = 2200;
unsigned int ph2 = 600;
unsigned int ph3 = 0;

int KV = 600;
int MaxV = 600;

unsigned int MA = 1000;


String ER0 = "0";
String ER1 = "0";
String ER2 = "0";
String ER3 = "0";
String ER4 = "0";
String ER5 = "0"; // err bit
String ER6 = "0";
String ER7 = "0";



// *******************************************************************
// *******************************************************************
// Function prototypes

// Flush the serial buffer
void serial_flush(void);

// read input commands
void readstring();
void handleInput(String inStr);
int extractNumber(String inStr);
bool containsStr(String inStr, String substr);
String MakeERString();
String PHString();
String MakeAMString();
String MakeATString();
float varyNumber(float target, float range);
void displaytime();
void readSDfile();
void writeSDfile(String toWrite);
String KO();
void sendCurrentShot();
void printhelp();



void setup() {
  Wire.begin();
  Serial.begin(19200);
  // put your setup code here, to run once:
  startMillis = millis();  //initial start time
  startMillis2 = millis();  //initial start time
  pinMode(7, OUTPUT);                                        
  
  Serial.println("Ready");
  currentMillis = 0;
  currentMillis2 = 0;
  randomSeed(analogRead(A0));
  
}
void loop() 
{
  float targetValue = 24.0;
  float variationRange = 3.0;

  float variedNumber = varyNumber(targetValue, variationRange);
  AT0 = (unsigned int)variedNumber;

  targetValue = 30.0;
  variationRange = 4;
  variedNumber = varyNumber(targetValue, variationRange);
  AT1 = (unsigned int)variedNumber;


  targetValue = 22.0;
  variationRange = 4;
  variedNumber = varyNumber(targetValue, variationRange);
  AT2 = (unsigned int)variedNumber;


  targetValue = 31.0;
  variationRange = 4;
  variedNumber = varyNumber(targetValue, variationRange);
  AT3 = (unsigned int)variedNumber;

  targetValue = 2200.0;
  variationRange = 40;
  variedNumber = varyNumber(targetValue, variationRange);
  AT4 = (unsigned int)variedNumber;

  targetValue = (float)MA;
  variationRange = 500;
  variedNumber = varyNumber(targetValue, variationRange);
  if(bShootingRays)
  {
    if(variedNumber > MA)
    {
      variedNumber = (float)MA;
    }
    AM1 = (unsigned int)variedNumber;
  }
  
  targetValue = (float)KV;
  variationRange = 10;
  variedNumber = varyNumber(targetValue, variationRange);
  if(bShootingRays)
  {
    if(variedNumber > KV)
    {
      variedNumber = (float)KV;
    }
    AM0 = (unsigned int)variedNumber;
  }

  readstring();
  currentMillis = millis();
  if(currentMillis - startMillis >= period)
  {

        if(bShootingRays)
        {
          Serial.println(KO());
        }
        if(autoSend)
        {
          sendCurrentShot();
        }
        updateActualKVandMAValues();
        
    
    startMillis = millis();
  }


  currentMillis2 = millis();
  if(currentMillis2 - startMillis2 >= period2)
  {
     
        startMillis2 = millis();
  }
}
void handleInput(String inStr)
{

      if(containsStr(inStr, "#PW"))
      {
        if(ER5 == "0" && (currentShot == ts1 || currentShot == ts2))
        {
          bPower = true;
        }
      }
      if(containsStr(inStr, "#ON"))
      {
        if((ER5 == "0" && (currentShot == ts1 || currentShot == ts2)) || (ER5 == "1" && (currentShot == ts1)))
        {
          bShootingRays = true;
          AM0 = KV;
          AM1 = MA;
          updateActualKVandMAValues();
        }
      }

      if(containsStr(inStr, "#ST"))
      {
         bPower = false;
         bShootingRays = false;
         currentShot = none;
         updateActualKVandMAValues();
      }

      if(containsStr(inStr, "?AT"))
      {
        Serial.println(MakeATString() );
      }

      if(containsStr(inStr, "?AM"))
      {
        Serial.println(MakeAMString());
      }
  
      if(containsStr(inStr, "#KV"))
      {
        
        int nKV = extractNumber(inStr);
        
        String sNKV = String(nKV);
        if(sNKV.length() >= 5)
        {
          String corection = sNKV.substring(0,4);
          nKV = extractNumber(corection);
        }      
        if(nKV > 599)
        {
          if(currentShot == ts1 && nKV > ph2)
          {
            ph2 = nKV;
            MaxV = nKV;
          }
          
          KV = nKV;
          Serial.print("!KV");
          Serial.println(KV);

          if(KV > ph2)
          {
            ER5 = "1";
            
          }else
          {
            ER5 = "0";
          }
          updateActualKVandMAValues();
        }
        
      }

      if(containsStr(inStr, "#FM2"))
      {
        // fan mode
        FMState = "!FM2";
      }
      
      if(containsStr(inStr, "#FM1"))
      {
        // fan mode
        FMState = "!FM1";
      }

      if(containsStr(inStr, "#FM0"))
      {
        // fan mode
        FMState = "!FM0";
      }

      if(containsStr(inStr, "#SETP")) // #ASF turns autosend off
      {
        period = extractNumber(inStr);
         Serial.print("!SETP");
         Serial.println(period);
      }

      if(containsStr(inStr, "?SETP")) // #ASF turns autosend off
      {
         Serial.print("!SETP");
         Serial.println(period);
      }

      if(containsStr(inStr, "#AST")) // #ASF turns autosend off
      {
       autoSend = true;
       Serial.println("!AST");
      }

      if(containsStr(inStr, "#ASF")) // #ASF turns autosend off
      {
       autoSend = false;
       Serial.println("!ASF");
      }

      if(containsStr(inStr, "#TS1"))
      {
          currentShot = ts1;
      }

      if(containsStr(inStr, "read"))
      {
         readSDfile();
      }

      if(containsStr(inStr, "?FM"))
      {
        Serial.println(FMState);
      }

      if(containsStr(inStr, "#TS2") && ER5 == "0")
      {
        currentShot = ts2;
      }
      if(containsStr(inStr, "#TS0"))
      {
        currentShot = none;
      }

      if(containsStr(inStr, "?TIME") || containsStr(inStr, "?time"))
      {
        Serial.print("!TIME ");
        displaytime();
      }

      if(containsStr(inStr, "?KV"))
      {
        Serial.print("!KV");
        Serial.println(KV);
      }
      if(containsStr(inStr, "#MA"))
      {
        int nMA = extractNumber(inStr);
        if(nMA > 599)
        {
          MA = nMA;
          Serial.print("!MA");
          Serial.println(MA);
        }
        updateActualKVandMAValues();
      }

      if(containsStr(inStr, "?MA"))
      {
        Serial.print("!MA");
        Serial.println(MA);
      }
      
      if(containsStr(inStr, "?PH"))
      {
          String nPH = PHString();
          Serial.println(nPH);
      }

      if(containsStr(inStr, "?ER"))
      {
          String nER = MakeERString();
          Serial.println(nER);
      }
    
      if(containsStr(inStr, "?TS"))
      {
          String nTS = "";
          if(currentShot == ts1)
          {
            nTS = "!TS1";
          }else  if(currentShot == ts2)
          {
            nTS = "!TS2";
          }else
          {
            nTS = "!TS0";
          }

          Serial.println(nTS + "\n");
      }


      stringComplete = false;
      inputString = "";
  
}
void sendCurrentShot()
{
  String nTS = "";
  if(currentShot == ts1)
  {
    nTS = "!TS1";
  }else  if(currentShot == ts2)
  {
    nTS = "!TS2";
  }else
  {
    nTS = "!TS0";
  }

  Serial.println(nTS);
  delay(iDelay);
  if(bPower)
  {
    Serial.println("PW ON");
  }else
  {
    Serial.println("PW OFF");
  }
  delay(iDelay);
  if(bShootingRays)
  {
    Serial.println("TUBE IS ON");
  }else
  {
    Serial.println("TUBE IS OFF");
  }
  delay(iDelay);

  Serial.println(FMState);

  delay(iDelay);

  Serial.print("Period: ");
  Serial.println(period);
  

  delay(iDelay);

}
bool containsStr(String inStr, String substr)
{
  //String substr = "#KV";
  int pos = inStr.indexOf(substr);

  if (pos >= 0) {
    return true;
  } else {
    return false;
  }
  delay(1000);
}

int extractNumber(String inStr)
{
  String numStr = "";
  
  // Extract numbers from the string
  for (int i = 0; i < inStr.length(); i++) {
    if (isdigit(inStr.charAt(i))) {
      numStr += inStr.charAt(i);
    }
  }

  // Convert the extracted string of numbers to an integer
  int num = atoi(numStr.c_str());
  
  return num;
}

















void printhelp()
{
  /*
    String help = "";

    help = "#PW - power";
    Serial.println(help);
    delay(iDelay);

    help = "#ON - on";
    Serial.println(help);
    delay(iDelay);

    help = "#ST - stop command";
    Serial.println(help);
    delay(iDelay);

    help = "?AT - Request Temps";
    Serial.println(help);
    delay(iDelay);
    
    help = "?AM - Request measured values";
    Serial.println(help);
    delay(iDelay);

    help = "#KV - Set KV";
    Serial.println(help);
    delay(iDelay);

    help = "#FM2 - Fan mode";
    Serial.println(help);
    delay(iDelay);

    help = "#AST - #ASF turns autosend off";
    Serial.println(help);
    delay(iDelay);

    help = "#ASF - #ASF turns autosend off";
    Serial.println(help);
    delay(iDelay);

    help = "#TS1 - Sets TS1";
    Serial.println(help);
    delay(iDelay);

    help = "read - reads values from SD card";
    Serial.println(help);
    delay(iDelay);

    help = "#TS2 - Sets TS2";
    Serial.println(help);
    delay(iDelay);

    help = "#TS0 - Sets TS0";
    Serial.println(help);
    delay(iDelay);

    help = "?TIME - Request time";
    Serial.println(help);
    delay(iDelay);

    help = "?time - Request time";
    Serial.println(help);
    delay(iDelay);

    help = "?KV - Request KV";
    Serial.println(help);
    delay(iDelay);

    help = "#MA - Set MA";
    Serial.println(help);
    delay(iDelay);

    help = "?MA - Request MA";
    Serial.println(help);
    delay(iDelay);

    help = "?PH - Request PH Values";
    Serial.println(help);
    delay(iDelay);

    help = "?ER - Request Errors";
    Serial.println(help);
    delay(iDelay);

    help = "?TS - Request TS setting";
    Serial.println(help);
    delay(iDelay);

    help = "?FM - Request Fan mode setting";
    Serial.println(help);
    delay(iDelay);

    help = "";
    */
}

float varyNumber(float target, float range) {
  float randomValue = (float)random(0, 1000) / 1000.0;
  float result = target - range + randomValue * (2 * range);
  return result;
}

void serial_flush(void) {
  while (Serial.available()) Serial.read();
}
void readstring()
{
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString += inChar;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (inChar == '\n')
        {
          handleInput(inputString);
          stringComplete = true;
          serial_flush();
          return;
        }
    }
}

String MakeATString()
{

  String sAT0 = String(AT0);
  int at0len = sAT0.length();
  int at0toadd = 3 - at0len;

  for(int i = 0;i < at0toadd;i++)
  {
    sAT0 = "0" + sAT0;
  }
  
  String sAT1 = String(AT1);
  int at1len = sAT1.length();
  int at1toadd = 3 - at1len;

  for(int i = 0;i < at1toadd;i++)
  {
    sAT1 = "0" + sAT1;
  }

   String sAT2 = String(AT2);
  int at2len = sAT2.length();
  int at2toadd = 3 - at2len;

  for(int i = 0;i < at2toadd;i++)
  {
    sAT2 = "0" + sAT2;
  }

  String sAT3 = String(AT3);
  int at3len = sAT3.length();
  int at3toadd = 3 - at3len;

  for(int i = 0;i < at3toadd;i++)
  {
    sAT3 = "0" + sAT3;
  }
  
  String newAT = "!AT" + sAT0 + "," + sAT1 + "," + sAT2 + "," + sAT3 + "," + String(AT4) + "<";
  return newAT;
}
String MakeAMString()
{

  String sAM0 = String(AM0);
  int am0len = sAM0.length();
  int amtoadd = 4 - am0len;

  for(int i = 0;i < amtoadd;i++)
  {
    sAM0 = "0" + sAM0;
  }
  
  String sAM1 = String(AM1);
  int am1len = sAM1.length();
  int am1toadd = 4 - am1len;

  for(int i = 0;i < am1toadd;i++)
  {
    sAM1 = "0" + sAM1;
  }

  String sAM2 = String(AM2);
  int am2len = sAM2.length();
  int am2toadd = 4 - am2len;

  for(int i = 0;i < am2toadd;i++)
  {
    sAM2 = "0" + sAM2;
  }

  String sAM3 = String(AM3);
  int am3len = sAM3.length();
  int am3toadd = 4 - am3len;

  for(int i = 0;i < am3toadd;i++)
  {
    sAM3 = "0" + sAM3;
  }
  
  String newAM = "!AM" + sAM0 + "," + sAM1 + "," + sAM2 + "," + sAM3 + "," + "0" + String(AM4) + "," + String(AM5) + "<";
  return newAM;
}
String MakeERString()
{
  String newER = "!ER" + ER0 + "," + ER1+ "," + ER2+ "," + ER3+ "," + ER4 + "," + ER5 + "," + ER6 + "," + ER7 + "," + "<";
  return newER;
}
String PHString()
{
  String newPH = "!PH" + String(ph1) + "," + String(ph2) + "," + String(ph3) + "<\n";
  return newPH;
}

String KO()
{
  String newPH = "#KO<\n";
  return newPH;
}

void updateActualKVandMAValues()
{
  if((currentShot == ts2 && bShootingRays) || (currentShot == ts1 && bShootingRays))
  {
    AM0 = KV;
    AM1 = MA; 
    digitalWrite(7, HIGH);
  }
  else if(currentShot == none)
  {
    AM0 = 0;
    AM1 = 0;
    digitalWrite(7, LOW);
  }else if(ER5 == "1" && (currentShot == ts2))
  {
    currentShot = none;
    bShootingRays = false;
    bPower = false;
    AM0 = 0;
    AM1 = 0;
    digitalWrite(7, LOW);
  }
  
}

void displaytime()
{
  Serial.print(myRTC.getYear(), DEC);
  Serial.print("-");
  Serial.print(myRTC.getMonth(century), DEC);
  Serial.print("-");
  Serial.print(myRTC.getDate(), DEC);
  Serial.print(" ");
  Serial.print(myRTC.getHour(h12Flag, pmFlag), DEC); //24-hr
  Serial.print(":");
  Serial.print(myRTC.getMinute(), DEC);
  Serial.print(":");
  Serial.println(myRTC.getSecond(), DEC);
}

void readSDfile()
{
  
  //SD.begin(4);
  
  //File myFile;
  
   //myFile = SD.open("test.txt");

   /*
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    
    
  } else {
    // if the file didn't open, print an error:
   Serial.println("error opening test.txt");
  }
  */


  //myFile.close();
  //SD.end();
  
}

void writeSDfile(String toWrite)
{
  /*
  SD.begin(4);
  File myFile;
   myFile = SD.open("test.txt",FILE_WRITE);
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.println(toWrite);
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  SD.end();
  */
}
