/*                          _                                                      _      
                           | |                                                    | |     
  ___ _ __ ___   ___  _ __ | |__   __ _ ___  ___       _ __   __ _ _ __   ___   __| | ___ 
 / _ \ '_ ` _ \ / _ \| '_ \| '_ \ / _` / __|/ _ \     | '_ \ / _` | '_ \ / _ \ / _` |/ _ \
|  __/ | | | | | (_) | | | | |_) | (_| \__ \  __/  _  | | | | (_| | | | | (_) | (_| |  __/
 \___|_| |_| |_|\___/|_| |_|_.__/ \__,_|___/\___| (_) |_| |_|\__,_|_| |_|\___/ \__,_|\___|
                                                                                          
*/
//--------------------------------------------------------------------------------------
// Relay's data recieved by emontx up to emoncms
// Relay's data recieved by emonglcd up to emoncms
// Decodes reply from server to set software real time clock
// Relay's time data to emonglcd - and any other listening nodes.
// Looks for 'ok' reply from http request to verify data reached emoncms

// emonBase Documentation: http://openenergymonitor.org/emon/emonbase

// Authors: Trystan Lea and Glyn Hudson
// Part of the: openenergymonitor.org project
// Licenced under GNU GPL V3
//http://openenergymonitor.org/emon/license

// EtherCard Library by Jean-Claude Wippler and Andrew Lindsay
// JeeLib Library by Jean-Claude Wippler
//
// THIS SKETCH REQUIRES:
//
// Libraries in the standard arduino libraries folder:
//	- JeeLib		https://github.com/jcw/jeelib
//	- EtherCard		https://github.com/jcw/ethercard/
//
// Other files in project directory (should appear in the arduino tabs above)
//	- decode_reply.ino
//	- dhcp_dns.ino
//--------------------------------------------------------------------------------------

#define DEBUG     //comment out to disable serial printing to increase long term stability 
#define UNO       //anti crash wachdog reset only works with Uno (optiboot) bootloader, comment out the line if using delianuova

#include <JeeLib.h>	     //https://github.com/jcw/jeelib
#include <EtherCard.h>		//https://github.com/jcw/ethercard 
#include <avr/wdt.h>

#define MYNODE 15            
#define freq RF12_868MHZ     // frequency
#define group 173            // network group
#define APIKEY "5ad2b3b67920d2b2eb4af72eb0a9d9e0"

// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x42,0x31,0x42,0x21,0xa2,0x2b };

byte Ethernet::buffer[700];
static uint32_t timer;

//Domain name of remote webserver - leave blank if posting to IP address 
char website[] PROGMEM = "www.emoncms.org";

//---------------------------------------------------
// Data structures for transfering data between units
//---------------------------------------------------

typedef struct { int realPower, powerFactor, Vrms, temperature; } PayloadTX;
PayloadTX emontx;

typedef struct { int temperature; } PayloadGLCD;
PayloadGLCD emonglcd;

typedef struct { int hour, mins;} PayloadBase;
PayloadBase emonbase;
//---------------------------------------------------

//---------------------------------------------------------------------
// The PacketBuffer class is used to generate the json string that is send via ethernet - JeeLabs
//---------------------------------------------------------------------
class PacketBuffer : public Print {
public:
    PacketBuffer () : fill (0) {}
    const char* buffer() { return buf; }
    byte length() { return fill; }
    void reset()
    { 
      memset(buf,NULL,sizeof(buf));
      fill = 0; 
    }
    virtual size_t write (uint8_t ch)
        { if (fill < sizeof buf) buf[fill++] = ch; }
    byte fill;
    char buf[150];
    private:
};
PacketBuffer str;

const int redLED = 6;                     // NanodeRF RED indicator LED
const int greenLED = 5;                   // NanodeRF GREEN indicator LED

int ethernet_error = 0;                   // Etherent (controller/DHCP) error flag
int rf_error = 0;                         // RF error flag - high when no data received 
int ethernet_requests = 0;                // count ethernet requests without reply                 

int dhcp_status = 0;
int dns_status = 0;
int emonglcd_rx = 0;                      // Used to indicate that emonglcd data is available
int data_ready=0;                         // Used to signal that emontx data is ready to be sent
unsigned long last_rf;                    // Used to check for regular emontx data - otherwise error

char line_buf[50];                        // Used to store line of http reply header

//-----------------------------------------------------------------------------------
// Ethernet callback
// recieve reply and decode
//-----------------------------------------------------------------------------------
static void my_callback (byte status, word off, word len) {
  get_header_line(2,off);      // Get the date and time from the header
  Serial.print("ok recv from server | ");    // Print out the date and time
  Serial.println(line_buf);    // Print out the date and time
  
  // Decode date time string to get integers for hour, min, sec, day
  // We just search for the characters and hope they are in the right place
  char val[1];
  val[0] = line_buf[23]; val[1] = line_buf[24];
  int hour = atoi(val);
  val[0] = line_buf[26]; val[1] = line_buf[27];
  int mins = atoi(val);
  val[0] = line_buf[29]; val[1] = line_buf[30];
  int sec = atoi(val);
  val[0] = line_buf[11]; val[1] = line_buf[12];
  int day = atoi(val);
    
  if (hour>0 || mins>0 || sec>0) {  //don't send all zeros, happens when server failes to returns reponce to avoide GLCD getting mistakenly set to midnight
	emonbase.hour = hour;              //add current date and time to payload ready to be sent to emonGLCD
  	emonbase.mins = mins;
  }
  //-----------------------------------------------------------------------------
  
  delay(100);
  
  // Send time data
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}    // if can send - exit if it gets stuck, as it seems too
  rf12_sendStart(0, &emonbase, sizeof emonbase);                        // send payload
  rf12_sendWait(0);
  
  Serial.println("time sent to emonGLCD");
  
  get_reply_data(off);
  if (strcmp(line_buf,"ok")) {ethernet_requests = 0; ethernet_error = 0;}  // check for ok reply from emoncms to verify data post request
 }

//**********************************************************************************************************************
// SETUP
//**********************************************************************************************************************
void setup () {
  
  //Nanode RF LED indictor  setup - green flashing means good - red on for a long time means bad! 
  //High means off since NanodeRF tri-state buffer inverts signal 
  pinMode(redLED, OUTPUT); digitalWrite(redLED,LOW);            
  pinMode(greenLED, OUTPUT); digitalWrite(greenLED,LOW);       
  delay(100); digitalWrite(redLED,HIGH);                          // turn off redLED
  
  Serial.begin(9600);
  Serial.println("\n[emonBase_MJW]");

  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {
    Serial.println( "Failed to access Ethernet controller");
    ethernet_error = 1;  
  }
  else {
    Serial.print("Ethernet MAC: ");
    for (byte i = 0; i < 6; ++i) {
      Serial.print(mymac[i], HEX);
      if (i < 5)
        Serial.print(":");
    }
  }
  Serial.println();

  dhcp_status = 0;
  dns_status = 0;
  ethernet_requests = 0;
  ethernet_error=0;
  rf_error=0;
    
  rf12_initialize(MYNODE, freq,group);
  last_rf = millis()-40000;                                       // setting lastRF back 40s is useful as it forces the ethernet code to run straight away
   
  digitalWrite(greenLED,HIGH);                                    // Green LED off - indicate that setup has finished 
 
  #ifdef UNO
  wdt_enable(WDTO_8S); 
  #endif;
}

//**********************************************************************************************************************
// LOOP
//**********************************************************************************************************************
void loop () {
  
  #ifdef UNO
  wdt_reset();
  #endif

  dhcp_dns(); // handle dhcp and dns setup - see dhcp_dns tab
  
  // Display error states on status LED
  if (ethernet_error==1 || rf_error==1 || ethernet_requests > 0) digitalWrite(redLED,LOW);
    else digitalWrite(redLED,HIGH);

  //-----------------------------------------------------------------------------------------------------------------
  // 1) On RF recieve
  //-----------------------------------------------------------------------------------------------------------------
  if (rf12_recvDone()){      
      if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)
      {
        int node_id = (rf12_hdr & 0x1F);
        
        if (node_id == 10)                                               // EMONTX
        {
          emontx = *(PayloadTX*) rf12_data;                              // get emontx data
          Serial.println();                                              // print emontx data to serial
          Serial.println("emonTx data rx");
          last_rf = millis();                                            // reset lastRF timer
          
          delay(50);                                                     // make sure serial printing finished
                               
          // JSON creation: JSON sent are of the format: {key1:value1,key2:value2} and so on
          
          str.reset();                                                   // Reset json string      
          str.print("{rf_fail:0");                                       // RF recieved so no failure
          
          // Add data from emontx
          str.print(",realPower:");        str.print(emontx.realPower);
          str.print(",powerFactor:");        str.print(emontx.powerFactor);
          str.print(",Vrms:");        str.print(emontx.Vrms/100.0);
          str.print(",temp_emontx:");        str.print(emontx.temperature/100.0);
    
          data_ready = 1;                                                // data is ready
          rf_error = 0;
        }
        
        if (node_id == 20)                                               // EMONGLCD 
        {
          emonglcd = *(PayloadGLCD*) rf12_data;                          // get emonglcd data
          Serial.print("emonGLCD temp recv: ");                          // print output
          Serial.println(emonglcd.temperature);  
          emonglcd_rx = 1;        
        }
      }
    }

  //-----------------------------------------------------------------------------------------------------------------
  // 2) If no data is recieved from rf12 module the server is updated every 30s with RFfail = 1 indicator for debugging
  //-----------------------------------------------------------------------------------------------------------------
  if ((millis()-last_rf)>30000)
  {
    last_rf = millis();                                                 // reset lastRF timer
    str.reset();                                                        // reset json string
    str.print("{rf_fail:1");                                            // No RF received in 30 seconds so send failure 
    data_ready = 1;                                                     // Ok, data is ready
    rf_error=1;
  }


  //-----------------------------------------------------------------------------------------------------------------
  // 3) Send data via ethernet
  //-----------------------------------------------------------------------------------------------------------------
  ether.packetLoop(ether.packetReceive());
  
  if (millis() > timer) 
    timer = millis() + 10000;
  
  if (data_ready) {
    
    // include temperature data from emonglcd if it has been recieved
    if (emonglcd_rx) {
      str.print(",temperature:");  
      str.print(emonglcd.temperature/100.0);
      emonglcd_rx = 0;
    }
    
    str.print("}\0");  //  End of json string
    
    Serial.print("json: "); Serial.println(str.buf); // print to serial json string

    // Example of posting to emoncms v3 demo account goto http://vis.openenergymonitor.org/emoncms3 
    // and login with sandbox:sandbox
    // To point to your account just enter your WRITE APIKEY 
    ethernet_requests ++;
    
//    Stash::prepare(PSTR("GET http://www.emoncms.org/emoncms3/api/post.json?apikey=7299afd5abccff1bbebbe867ebe66958&json=$F"),str.buf);
  //  ether.tcpSend();
//    ether.browseUrl(PSTR("/foo/"), "bar", website, my_callback);

    ether.browseUrl(PSTR("/api/post.json?apikey=7299afd5abccff1bbebbe867ebe66958&json="),str.buf, website, my_callback);
//    ether.browseUrl(PSTR("/emoncms3/api/post.json?apikey=5ad2b3b67920d2b2eb4af72eb0a9d9e0&json="),str.buf, website, my_callback);
//    ether.httpPost(PSTR("/emoncms3/api/post.json?apikey=5ad2b3b67920d2b2eb4af72eb0a9d9e0&json=")
    data_ready =0;
  }
  
  if (ethernet_requests > 10) delay(10000); // Reset the nanode if more than 10 request attempts have been tried without a reply

}
//**********************************************************************************************************************
