#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include<Wire.h>

/* Bus arbitrator FSM states */
#define NoTriggerNoMaster 0     //idle state
#define AttemptToBeMaster 1     //we will attempt to claim master
#define TriggerIamMaster 2      //we are master after a successful claim
#define TriggerCollision 3      //both micro-controllers are masters, i.e. a collision
#define NoTriggerOtherMaster 6
#define OUT D1
#define OtherMASTER D5
#define LED D4
char state;
bool trigger;
byte Command = 0x00;
int i = 0;
char ACK[16];
String LEDvalue;


bool FirstTime = true;
bool OtherMaster() {
  return (digitalRead(OtherMASTER) != HIGH);
}

//module3
bool init_module3_clock;
unsigned long module3_time, module3_delay, module3_timestamp;
bool module3_doStep;
unsigned char module3_i;

long unsigned int timestamp;
// Give your access point a name and a password
const char* ssid = "Asia Access Point";
const char* password = "ouarda";

// Create instance of the ESP8266WebServer with a port number of
// 80 for HTTP.
ESP8266WebServer server(80);

// This is the 'root'/'index page. This can *typically* be
// accessed at http://192.168.4.1. This contains two radio
// buttons and a submit button
const char INDEX_HTML[] =
  "<!DOCTYPE HTML>"
  "<html>"
  "<head>"
  "<title>ESP8266 Webpage</title>"
  "<meta http-equiv = \"refresh\" content = \"5\">"
  "</head>"
  "<body>"
  "<h1> IOD_Experiment </h1>"

  "<style>"\
  "body { background-color: #FC9CF9; font-family: Arial, Helvetica, Sans-Serif; font-size: 1.2em;}"\
  "h1 { Color: #0000aa;}"\
  "</style>"\

"<FORM action=\"/\" method=\"post\">" "<b>" "LED<br>"
"<INPUT type=\"radio\" name=\"LED\" value=\"1\">Equal_priority<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"2\">Set1_priority<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"3\">Set2_priority<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"4\">Maintenance<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"5\">Pelican<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"6\">Barrier<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"7\">F1<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"8\">Reset<BR>"
"<INPUT type=\"radio\" name=\"LED\" value=\"9\">GetStatus<BR>"

  "<INPUT type=\"submit\" =\"Send\">" "</P>"
  "<OUTPUT type=\"recieve\" =\"Send\">" "</P>"
  "</FORM>"

  "</body>"
  "</html>";


// This is our '404' Error - Page Not Found page
const char NOTFOUND_HTML[] =
  "<!DOCTYPE HTML>"
  "<html>"
  "<head>"
  "<title>404 Not Found</title>"
  "<body>"
  "<h1>Oops! It's 404-time! Page not found!</h1>"
  "</body>"
  "</html>";


// This is our '500' Error - Internal Server Error
const char INTERNAL_SERVER_ERROR_HTML[] =
  "<!DOCTYPE HTML>"
  "<html>"
  "<head>"
  "<title>500 Internal Server Error</title>"
  "<body>"
  "<h1>500 Internal Server Error</h1>"
  "<p>LED mode unsupported<\p>"
  "</body>"
  "</html>";


//----------------------------------------------------------------------------
//                             PAGE HANDLING FUNCTIONS
//----------------------------------------------------------------------------

// ===============================================================
/// \fn     void handleRoot()
///
/// \brief  This function is called whenever a call to the index
///         page (i.e. http://192.168.4.1) is made.
///
///         If the LED name has a valid argument, the function
///         handleSubmit() is called to process the argument.
///         Otherwise, the response held in the INDEX_HTML will be
///         sent in text/html format with HTTP error code 200 (OK)
// ===============================================================
void handleRoot()
{

  char html[2048];
  char temp[80];
  char temp1[80];
  char temp2[80];
  char temp3[80];
  char temp4[80];
  char temp5[80];
  char temp6[80];
  char temp7[80];
  char temp8[80];
  char temp9[80];

  if ( LEDvalue == "9") {
    strcpy(html, INDEX_HTML);
    sprintf(temp, "<p>Status =%d</p>", ACK[0]);
    sprintf(temp1, "<p>CurrentCycle =%d</p>", ACK[1]);
    sprintf(temp2, "<p>NextCycle =%d</p>", ACK[2]);
    sprintf(temp3, "<p>LightStatus =%d</p>", ACK[3]);
    sprintf(temp4, "<p>SW1 waiting counts =%d</p>", ACK[4]);
    sprintf(temp5, "<p>SW2 waiting counts =%d</p>", ACK[5]);
    sprintf(temp6, "<p>total SW1 counts =%d</p>", ACK[6]);
    sprintf(temp7, "<p>total SW2 counts =%d</p>", ACK[7]);
    sprintf(temp8, "<p>TiltStatus =%d</p>", ACK[8]);
    sprintf(temp9, "<p>T =%d</p>", ACK[9]);

    strcat(html, temp);
    strcat(html, temp1);
    strcat(html, temp2);
    strcat(html, temp3);
    strcat(html, temp4);
    strcat(html, temp5);
    strcat(html, temp6);
    strcat(html, temp7);
    strcat(html, temp8);
    strcat(html, temp9);

    strcat(html, "</body>");
    strcat(html, "</html>");
    server.send(200, "text/html", html);
  }

  if (server.hasArg("LED"))
  {

    handleSubmit();
  }
  else
  {
    server.send(200, "text/html", INDEX_HTML);
    return;
  }
}


// ===============================================================
/// \fn     void handleSubmit()
///
/// \brief  This function is called to control the LED based on
///         the argument.
///
///         In the event of a valid argument, the LED is either
///         illuminated or extinguished. The response held in
///         INDEX_HTML will be sent in text/html format with
///         HTTP error code 200 (OK).
///         Otherwise, the internal server error response will be
///         sent in text/html format with HTTP error code 500 (OK)
// ===============================================================

void handleSubmit()
{

  trigger = true;


  if (!server.hasArg("LED"))
  {
    return;
  }


  // Obtain the argument
  LEDvalue = server.arg("LED");
  Serial.println(LEDvalue);
  // Act on it
  if (LEDvalue == "1" )
  {
    Serial.println("I AM HERE!");
    Command = 0x61;
    server.send(200, "text/html", INDEX_HTML);
    return;
  }
  else if (LEDvalue == "2")
  {
    Command = 0x62;
    server.send(200, "text/html", INDEX_HTML);
  }
  else if (LEDvalue == "3")
  {
    Command = 0x63;
    server.send(200, "text/html", INDEX_HTML);
  }
  else if (LEDvalue == "4")
  {
    Command = 0x64;
    server.send(200, "text/html", INDEX_HTML);
  }
  else if (LEDvalue == "5")
  {
    Command = 0x65;
    server.send(200, "text/html", INDEX_HTML);
  }
  else if (LEDvalue == "6")
  {
    Command = 0x66;
    server.send(200, "text/html", INDEX_HTML);

  }
  else if (LEDvalue == "7")
  {
    Command = 0x67;
    server.send(200, "text/html", INDEX_HTML);
  }
  else if (LEDvalue == "8")
  {
    Command = 0x68;
    server.send(200, "text/html", INDEX_HTML);
  }
  else if (LEDvalue == "9")
  {
    Command = 0x69;
    server.send(200, "text/html", INDEX_HTML);
  }
  else
  {
    server.send(500, "text/html", INTERNAL_SERVER_ERROR_HTML);
  }
}



// ===============================================================
/// \fn     void handleLEDon()
///
/// \brief  This function is called whenever a call is made for
///         "ledon" (i.e. http://192.168.4.1/ledon)
///
///         The LED will be illuminated.
///         The response held in the string INDEX_HTML will be
///         sent in text/html format with the error code 200 (OK)
// ===============================================================
void handleLEDon()
{
  writeLED(true);
  server.send(200, "text/html", INDEX_HTML);
}


// ===============================================================
/// \fn     void handleLEDoff()
///
/// \brief  This function is called whenever a call is made for
///         "ledoff" (i.e. http://192.168.4.1/ledoff)
///
///         The LED will be extinguished.
///         The response held in the string INDEX_HTML will be
///         sent in text/html format with the error code 200 (OK)
// ===============================================================
void handleLEDoff()
{
  writeLED(false);
  server.send(200, "text/html", INDEX_HTML);
}


// ===============================================================
/// \fn     void handleNotFound()
///
/// \brief  This function is called whenever a call is made for
///         an unsupported page
///
///         The response held in the string NOTFOUND_HTML will
///         be sent in text/html format with the error code 404
///         (PAGE NOT FOUND)
// ===============================================================
void handleNotFound()
{
  server.send(404, "text/html", NOTFOUND_HTML);
}


// ===============================================================
/// \fn     void writeLED(bool LEDon)
///
/// \brief  Function to control the LED
///
/// \param  LEDon     LED status
// ===============================================================
void writeLED(bool LEDon)
{
  // Note inverted logic for NodeMCU 12E ESP8266
  if (LEDon)
  {
    digitalWrite(BUILTIN_LED, 0);
  }
  else
  {
    digitalWrite(BUILTIN_LED, 1);
  }
}


// ===============================================================
/// \fn     void setup()
/// TESTING
/// \brief  One-off set-up calls.
// ===============================================================

void setup(void)
{
  // Set-up the LED GPIO
  pinMode(BUILTIN_LED, OUTPUT);
  writeLED(true);
  //    Wire.onReceive(receiveEvent);
  Wire.begin(D3, D6);

  Serial.begin(9600);
  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.println(""); Serial.print("Connected to ");
  Serial.println(ssid); Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  randomSeed(analogRead(A0));
  // Associate functions for root, ledon, ledoff and "Not Found"
  server.on("/", handleRoot);
  server.on("/ledoff", handleLEDoff);
  server.onNotFound(handleNotFound);
  server.begin();

  // Kick-start the server
  server.begin();
  init_module3_clock = false;

  Serial.print("Connect to http://");
  Serial.println(WiFi.softAPIP());
}


// ===============================================================
/// \fn     loop()
///
/// \brief  Run ad-infinitum
// ===============================================================
void loop(void)
{
  server.handleClient();
  switch (state) {
    case NoTriggerNoMaster: 
    digitalWrite(OUT, HIGH);
    digitalWrite(LED,HIGH);
    
    timestamp = millis() + random(10);
        if   (!trigger && !OtherMaster()) {
          state = NoTriggerNoMaster;
        }        
        if   (trigger  && !OtherMaster()) {
          state = AttemptToBeMaster;
        }          
        if   (!trigger && OtherMaster())  {
          state = NoTriggerOtherMaster;
        }     
        break;

    case NoTriggerOtherMaster: 
    digitalWrite(OUT, HIGH);
    digitalWrite(LED, HIGH);
   if   (!trigger && !OtherMaster()) {
          state = NoTriggerNoMaster;
        }        
        if   (trigger  && !OtherMaster()) {
          state = AttemptToBeMaster;
        }          
        if   (!trigger && OtherMaster())  {
          state = NoTriggerOtherMaster;
        }   
        if   (trigger  && OtherMaster())  {
          state = TriggerCollision;
        }       
        break;

    case AttemptToBeMaster: 
    digitalWrite(OUT, LOW);
    digitalWrite(LED, HIGH);
      if   (!trigger && !OtherMaster()) {
        state = NoTriggerNoMaster;
      }
      if   (!trigger && OtherMaster())  {
        state = NoTriggerOtherMaster;
      }     
      if   (trigger  && OtherMaster())  {
        state = TriggerCollision;
      }       
      else if (millis() < timestamp) state = AttemptToBeMaster;
           else {
                if  (!OtherMaster()) {state = TriggerIamMaster;}
                else {state = TriggerCollision;}
                }
        break;

    case TriggerIamMaster: 
    digitalWrite(OUT, LOW);
    digitalWrite(LED, LOW);
        if   (!trigger && !OtherMaster()) {
          state = NoTriggerNoMaster;
        }       
        if   (trigger  && !OtherMaster()) {
          state = TriggerIamMaster;
        }          
        if   (!trigger && OtherMaster())  {
          state = NoTriggerOtherMaster;
        }     
        if   (trigger  && OtherMaster())  {
          state = TriggerCollision;
        }        
        break;

    case TriggerCollision: 
    digitalWrite(OUT, HIGH);
    digitalWrite(LED, HIGH);
        if   (!trigger && !OtherMaster()) {
          state = NoTriggerNoMaster;
        }        
        if   (trigger  && !OtherMaster()) {
          state = AttemptToBeMaster;
        }         
        if   (!trigger && OtherMaster())  {
          state = NoTriggerOtherMaster;
        }     
        if   (trigger  && OtherMaster())  
             if (millis() < timestamp)
                 {state = TriggerCollision;break;} 
             else
                 {state = AttemptToBeMaster;}

     default:
      state = NoTriggerNoMaster;
      break;
}

  { // module 3
    /* test bench */
    if (init_module3_clock) {
      module3_delay = 1;
      module3_time = millis();
      module3_doStep = false;
      init_module3_clock = false;
      module3_i = 0;
    }
    else {
      unsigned long m;
      m = millis();
      if ((long(m - module3_time) > module3_delay)) {
        module3_time = m; module3_doStep = true;
      }
      else module3_doStep = false;
    }

    if (module3_doStep) {
      switch (module3_i) {
        case 0:
          module3_timestamp = millis() + 50;
        case 1:
          if ((long)(millis() - module3_timestamp) < 0)  module3_i = 1;
          else module3_i = 2;
          break;
        case 2:
          trigger = true;
        case 3:
          if (state != TriggerIamMaster) module3_i = 3;
          else module3_i = 4;
          break;
        case 4:
          {
            module3_timestamp = millis() + 850;
            Wire.requestFrom(8, 10);
            Wire.endTransmission();
          }
          if ((Command >= 0x61) && (Command <= 0x68))
          {
            Wire.beginTransmission(8);
            Wire.write(Command);
            Serial.print("SENDING......");
            Serial.println(Command);
            Command = 0x00;
            Wire.endTransmission();
          }

          else if ( Command == 0x69)
          {
            while ( Wire.available())
            {
              ACK[i] = Wire.read();
              i++;
            }
            Command = 0x00;
            i = 0;
          }

          {
            Serial.println(ACK[0]);
            Serial.println(ACK[1]);
            Serial.println(ACK[2]);
            Serial.println(ACK[3]);
            Serial.println(ACK[4]);
            Serial.println(ACK[5]);
            Serial.println(ACK[6]);
            Serial.println(ACK[7]);
            Serial.print( ACK[8]);
            Serial.print(ACK[9]);
            trigger = false;
            module3_i = 0;
          }
        default:
          trigger = false;
          module3_i = 0;

      }
    }
  }
}
