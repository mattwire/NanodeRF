/*                          _                                                      _      
                           | |                                                    | |     
  ___ _ __ ___   ___  _ __ | |__   __ _ ___  ___       _ __   __ _ _ __   ___   __| | ___ 
 / _ \ '_ ` _ \ / _ \| '_ \| '_ \ / _` / __|/ _ \     | '_ \ / _` | '_ \ / _ \ / _` |/ _ \
|  __/ | | | | | (_) | | | | |_) | (_| \__ \  __/  _  | | | | (_| | | | | (_) | (_| |  __/
 \___|_| |_| |_|\___/|_| |_|_.__/ \__,_|___/\___| (_) |_| |_|\__,_|_| |_|\___/ \__,_|\___|
                                                                                          
*/
//--------------------------------------------------------------------------------------
// Relay's data recieved by emontx up to emoncms and pachube
// Decode reply from server to set software real time clock and check for server 'ok'
// emonBase Documentation: http://openenergymonitor.org/emon/emonbase
// Authors: Trystan Lea, Glyn Hudson and Francois Hug
// Part of the: openenergymonitor.org project
// Licenced under GNU GPL V3
// http://openenergymonitor.org/emon/license
// EtherCard Library by Jean-Claude Wippler and Andrew Lindsay
// JeeLib Library by Jean-Claude Wippler
//--------------------------------------------------------------------------------------

#define DEBUG     //comment out to disable serial printing to increase long term stability 
#define UNO       //anti crash wachdog reset only works with Uno (optiboot) bootloader, comment out the line if using delianuova

#define HTTP_TIMEOUT 10000	// time to wait for server reply (in ms)

#define POST2EMONCMS
//#define POST2LOCAL
#define POST2PACHUBE

#include <Wire.h>
#include <RTClib.h>
RTC_Millis RTC;

#include <JeeLib.h>	     //https://github.com/jcw/jeelib
#include <avr/wdt.h>

#include <BMP085.h>          // https://github.com/adafruit/BMP085-Library
BMP085 pressureSens;
float Temp085;
unsigned long Pressure; 

#define MYNODE 35            // node ID 30 reserved for base station
#define freq RF12_433MHZ     // frequency
#define group 0xb3            // network group 

// The RF12 data payload - a neat way of packaging data when sending via RF - JeeLabs
// must be same structure as transmitted from emonTx
typedef struct {
	byte boardStatus;
	int temperature;
	int humidity;		     // humidity
	int light;
	int battery;
} Payload;
Payload emontx;     

//---------------------------------------------------------------------
// The PacketBuffer class is used to generate the json string that is send via ethernet - JeeLabs
//---------------------------------------------------------------------
class PacketBuffer : public Print {
	public:
		PacketBuffer () : fill (0) {}
		const char* buffer() { return buf; }
		byte length() { return fill; }
		void reset() { 
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

//--------------------------------------------------------------------------
// Ethernet
//--------------------------------------------------------------------------
#include <EtherCard.h>		//https://github.com/jcw/ethercard 
#include <NanodeMAC.h>          // https://github.com/thiseldo/NanodeMAC

// ethernet interface mac address, must be unique on the LAN
static uint8_t mymac[6] = { 0,0,0,0,0,0 };
NanodeMAC mac( mymac );
byte Ethernet::buffer[500];
static uint32_t timer;

//Domain name of remote webservers - leave blank if posting to IP address 
#ifdef POST2EMONCMS
	char websiteemon[] PROGMEM = "vis.openenergymonitor.org";
	static byte emonip[] = { 0,0,0,0 };
	static uint16_t emonport = 80;
	#define APIKEY_EMON "/emoncms3/api/post.json?apikey=EMONCMS_WRITE_APIKEY&json="
#endif

#ifdef POST2LOCAL
	char websitelocal[] PROGMEM = "";
	static byte hislocalip[] = { 192,168,1,105 };  // set it to local IP of emoncms
	static uint16_t localport = 8888;
	#define APIKEY_LOCAL "/emoncms3/api/post.json?apikey=EMONCMS_WRITE_APIKEY&json="
#endif

#ifdef POST2PACHUBE
	char websitepac[] PROGMEM = "api.pachube.com";
	static byte pachubeip[] = { 0,0,0,0 };
	static uint16_t pachubeport = 80;
	// Pachube change these settings to match your own setup
	#define FEED_PAC  "/v2/feeds/FEED_NUMBER.csv?_method=put"
	#define APIKEY_PAC "X-PachubeApiKey: PACHUBE_WRITE_APIKEY"
#endif

//--------------------------------------------------------------------------
// Flow control varaiables
int dataReady=0;                                                  // is set to 1 when there is data ready to be sent
unsigned long lastRF;                                             // used to check for RF recieve failures
unsigned long lastBMP085;                                         // used to check for BMP085 measurements
MilliTimer tReply;
static boolean httpHaveReply;
byte emontx_nodeID;

//NanodeRF error indication LED variables 
const int redLED=6;                      //NanodeRF RED indicator LED
const int greenLED=5;                    //NanodeRF GREEN indicator LED
int error=0;                             //Etherent (controller/DHCP) error flag
int RFerror=0;                           //RF error flag - high when no data received 
int dhcp_status = 0;
#ifdef POST2EMONCMS
	int dns_status_emon = 0; 
#endif
#ifdef POST2PACHUBE
	int dns_status_pac = 0; 
#endif
int request_attempt = 0;
char line_buf[50];

//-----------------------------------------------------------------------------------
// Ethernet callback
// recieve reply and decode
//-----------------------------------------------------------------------------------
#ifdef POST2PACHUBE
	static void callback_pac (byte status, word off, word len) {
		get_header_line(1,off);      // Get the http status code
		#ifdef DEBUG
			Serial.println(line_buf);    // Print out the http status code
		#endif
		//-----------------------------------------------------------------------------
		if (strcmp(line_buf,"HTTP/1.1 200 OK")) {
		#ifdef DEBUG
			Serial.println("ok received from pachube");
		#endif
		httpHaveReply = 1;
		request_attempt = 0;
		error=0;
		}
	}
	
	static void format_pac_json (void) {
		str.reset();                                                 // Reset json string      
		//str.println("RF,0");                                     // RF recieved so no failure
		//str.print("Sta11,");    str.println(emontx.boardStatus);
		str.print("Tout,");    str.println(((float)emontx.temperature)/100);
		str.print("Hout,");    str.println(((float)emontx.humidity)/10);
		str.print("Light,");    str.println(emontx.light);
		//str.print("Bat,");    str.println(((float)emontx.battery)/1000);
		str.print("Press,");    str.println(((float)Pressure)/100);
		str.print("Tin,");    str.println(Temp085);
	}
#endif

#ifdef POST2EMONCMS
	static void callback_emon (byte status, word off, word len) {
		get_header_line(1,off);      // Get the http status code
		#ifdef DEBUG
			Serial.println(line_buf);    // Print out the http status code
		#endif
		//-----------------------------------------------------------------------------
		get_reply_data(off);
		//#ifdef DEBUG
		//Serial.println(line_buf);
		//#endif
		if (strcmp(line_buf,"ok")) {
			#ifdef DEBUG
				Serial.println("ok received from emoncms");
			#endif
			httpHaveReply = 1;
			request_attempt = 0;
			error=0;
		}
	}
#endif

#if defined(POST2EMONCMS) || defined(POST2LOCAL)
	static void format_emon_json (void) {
		// JSON creation: JSON sent are of the format: {key1:value1,key2:value2} and so on
		str.reset();                                                 // Reset json string      
		str.print("{rf_fail:0");                                     // RF recieved so no failure
		str.print(",sta11:");    str.print(emontx.boardStatus);
		str.print(",temp1:");    str.print(emontx.temperature);
		str.print(",hum1:");    str.print(emontx.humidity);
		str.print(",light:");    str.print(emontx.light);
		str.print(",battery:");    str.print(emontx.battery);
		str.print(",pressure:");    str.print(Pressure);
		str.print(",temp085:");    str.print(Temp085*100);
		str.print("}\0");
	}
#endif

#ifdef POST2LOCAL
	static void callback_local (byte status, word off, word len) {  // update RTC time with local server time
		get_header_line(1,off);      // Get the http status code
		#ifdef DEBUG
			Serial.println(line_buf);    // Print out the http status code
		#endif
		get_header_line(2,off);      // Get the date and time from the header
		#ifdef DEBUG
			Serial.println(line_buf);    // Print out the date and time
		#endif
		// Decode date time string to get integers for hour, min, sec, day
		// We just search for the characters and hope they are in the right place
		/*
		char val[1];
		val[0] = line_buf[23]; val[1] = line_buf[24];
		int hour = atoi(val);
		val[0] = line_buf[26]; val[1] = line_buf[27];
		int mins = atoi(val);
		val[0] = line_buf[29]; val[1] = line_buf[30];
		int sec = atoi(val);
		val[0] = line_buf[11]; val[1] = line_buf[12];
		int day = atoi(val);
		// Set the RTC
		RTC.adjust(DateTime(2012, 2, day, hour, mins, sec));
		DateTime now = RTC.now();
		*/
		//-----------------------------------------------------------------------------
		get_reply_data(off);
		//#ifdef DEBUG
		//Serial.println(line_buf);
		//#endif
		if (strcmp(line_buf,"ok")) {
			#ifdef DEBUG
				Serial.println("ok received from local emon");
			#endif
			httpHaveReply = 1;
			request_attempt = 0;
			error=0;
			}
		}
		static void format_local_json (void) {
		format_emon_json();
	}
#endif

//**********************************************************************************************************************
// SETUP
//**********************************************************************************************************************
void setup () {
	//Nanode RF LED indictor  setup - green flashing means good - red on for a long time means bad! 
	//High means off since NanodeRF tri-state buffer inverts signal 
	pinMode(redLED, OUTPUT); digitalWrite(redLED,LOW);            
	pinMode(greenLED, OUTPUT); digitalWrite(greenLED,LOW);       
	delay(100); digitalWrite(redLED,HIGH);                        //turn off redLED
	#ifdef DEBUG
		Serial.begin(9600);
		Serial.println("\n[webClient riaDesign]");
	#endif
	error=0;
	if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {
		#ifdef DEBUG
			Serial.println( "Failed to access Ethernet controller");
		#endif
		error=1;  
	}
	dhcp_status = 0;
	httpHaveReply = 0;
	#ifdef POST2EMONCMS
		int dns_status_emon = 0; 
	#endif
	#ifdef POST2PACHUBE
		int dns_status_pac = 0; 
	#endif
	request_attempt = 0;
	error=0;
	rf12_initialize(MYNODE, freq, group);
	lastRF = millis();
	lastBMP085 = millis()-60000;
	pressureSens.begin();
	digitalWrite(greenLED,HIGH);                                    //Green LED off - indicate that setup has finished 
	#ifdef UNO
		wdt_enable(WDTO_8S); 
	#endif
}
//**********************************************************************************************************************


//**********************************************************************************************************************
// LOOP
//**********************************************************************************************************************
void loop () {
	#ifdef UNO
		wdt_reset();
	#endif
	//-----------------------------------------------------------------------------------
	// Get DHCP address
	// Putting DHCP setup and DNS lookup in the main loop allows for: 
	// powering nanode before ethernet is connected
	//-----------------------------------------------------------------------------------
	if (ether.dhcpExpired()) dhcp_status = 0;    // if dhcp expired start request for new lease by changing status
	if (!dhcp_status){
		#ifdef UNO
			wdt_disable();
		#endif 
		dhcp_status = ether.dhcpSetup();           // DHCP setup
		#ifdef UNO
			wdt_enable(WDTO_8S);
		#endif
		#ifdef DEBUG
			Serial.print("DHCP status: ");             // print
			Serial.println(dhcp_status);               // dhcp status
		#endif
		if (dhcp_status){                          // on success print out ip's
			ether.printIp("IP:  ", ether.myip);
			ether.printIp("GW:  ", ether.gwip);  
			//static byte dnsip[] = {8,8,8,8};  
			static byte dnsip[] = {192,168,1,1};
			ether.copyIp(ether.dnsip, dnsip);
			ether.printIp("DNS: ", ether.dnsip);          
		} else { error=1; }   
	}

	//-----------------------------------------------------------------------------------
	// Get server address via DNS
	//-----------------------------------------------------------------------------------
	#ifdef POST2EMONCMS
		if (dhcp_status && !dns_status_emon){
			#ifdef UNO
				wdt_disable();
			#endif 
			dns_status_emon = ether.dnsLookup(websiteemon);    // Attempt DNS lookup
			#ifdef UNO
				wdt_enable(WDTO_8S);
			#endif
			#ifdef DEBUG
				Serial.print("DNS status emon: ");             // print
				Serial.println(dns_status_emon);               // dns status
			#endif
			if (dns_status_emon){
				ether.copyIp(emonip, ether.hisip); 
				#ifdef DEBUG
					ether.printIp("SRV emoncms: ", emonip);         // server ip
				#endif
			} else { error=1; }  
		}
	#endif
	
	#ifdef POST2PACHUBE
		if (dhcp_status && !dns_status_pac){
			#ifdef UNO
				wdt_disable();
			#endif 
			dns_status_pac = ether.dnsLookup(websitepac);    // Attempt DNS lookup
			#ifdef UNO
				wdt_enable(WDTO_8S);
			#endif
			#ifdef DEBUG
				Serial.print("DNS status pachube: ");             // print
				Serial.println(dns_status_pac);               // dns status
			#endif
			if (dns_status_pac) {
				ether.copyIp(pachubeip, ether.hisip); 
				#ifdef DEBUG
					ether.printIp("SRV pachube: ", pachubeip);         // server ip
				#endif
			} else { error=1; }  
		}
	#endif
  
	if (error==1 || RFerror==1 || request_attempt > 1) digitalWrite(redLED,LOW);      //turn on red LED if RF / DHCP or Etherent controllor error. Need way to notify of server error
	else digitalWrite(redLED,HIGH);

	//---------------------------------------------------------------------
	// On data receieved from rf12
	//---------------------------------------------------------------------
	if (rf12_recvDone() && rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0) {
		digitalWrite(greenLED,LOW);                                   // turn green LED on to indicate RF recieve 
		emontx=*(Payload*) rf12_data;                                 // Get the payload
		emontx_nodeID = rf12_hdr & 0x1F;                                //extract node ID from received packet - only needed when multiple emonTx are posting on same network 
		RFerror=0;                                                    //reset RF error flag
		if (emontx_nodeID == 0x11) {
			dataReady = 1;                                                // Ok, data is ready
		}
		lastRF = millis();                                            // reset lastRF timer
		digitalWrite(greenLED,HIGH);                                  // Turn green LED on OFF
		#ifdef DEBUG 
			Serial.println("RF recieved");
		#endif
	}
  
	// If no data is recieved from rf12 module the server is updated every 11s with RFfail = 1 indicator for debugging
	if ((millis()-lastRF)>601000) {
		lastRF = millis();                                            // reset lastRF timer
		dataReady = 1;                                                // Ok, data is ready
		RFerror=1;
	}

	// update BMP085 pressure and temperature every 60s
	if ((millis()-lastBMP085)>60000) {
		Temp085 = pressureSens.readTemperature();
		Pressure = pressureSens.readPressure();
		lastBMP085 = millis();
	}

	ether.packetLoop(ether.packetReceive());
  
	if (dataReady) {
		#ifdef POST2LOCAL
			while (ether.packetLoop(ether.packetReceive()) != 0) {}
			ether.copyIp(ether.hisip, hislocalip); 
			ether.hisport = localport;
			httpHaveReply = 0;
			if(RFerror) {
				str.reset();                                                  // reset json string
				str.print("{rf_fail:1}");                                    // No RF received in 30 seconds so send failure 
			} else {
				format_local_json();
			}
			#ifdef DEBUG 
				Serial.println(str.buf); 
				Serial.println(request_attempt);   
			#endif    // Print final json string to terminal
			ether.browseUrl(PSTR(APIKEY_LOCAL),str.buf, websitelocal, callback_local);
			// Wait for reply
			tReply.set(HTTP_TIMEOUT);
			#ifdef UNO
				wdt_disable();
			#endif 
			while (!httpHaveReply) {
				ether.packetLoop(ether.packetReceive());
				if (tReply.poll()) {
					error=1;        // network timeout
					break;
				}
			}
			#ifdef UNO
				wdt_enable(WDTO_8S);
			#endif
			ether.packetLoop(ether.packetReceive());
		#endif
	
		#ifdef POST2EMONCMS
			if (emontx_nodeID == 0x11) {
				while (ether.packetLoop(ether.packetReceive()) != 0) {}
				ether.copyIp(ether.hisip, emonip); 
				ether.hisport = emonport;
				httpHaveReply = 0;
				if(RFerror) {
					str.reset();                                                  // reset json string
					str.print("{rf_fail:1}");                                    // No RF received in 30 seconds so send failure 
				} else {
					format_emon_json();
				}
				#ifdef DEBUG 
					Serial.println(str.buf); 
					Serial.println(request_attempt);   
				#endif    // Print final json string to terminal
				ether.browseUrl(PSTR(APIKEY_EMON),str.buf, websiteemon, callback_emon);
				// Wait for reply
				tReply.set(HTTP_TIMEOUT);
				#ifdef UNO
					wdt_disable();
				#endif 
				while (!httpHaveReply) {
					ether.packetLoop(ether.packetReceive());
					if (tReply.poll()) {
						error=1;        // network timeout
						break;
					}
				}
				#ifdef UNO
					wdt_enable(WDTO_8S);
				#endif
				ether.packetLoop(ether.packetReceive());
			}
		#endif
	
		#ifdef POST2PACHUBE
			if ((!RFerror) && (emontx_nodeID == 0x11)) {
				while (ether.packetLoop(ether.packetReceive()) != 0) {}
				ether.copyIp(ether.hisip, pachubeip); 
				ether.hisport = pachubeport;
				httpHaveReply = 0;
				format_pac_json();
				#ifdef DEBUG 
					Serial.println(str.buf); 
					Serial.println(request_attempt);   
				#endif    // Print final json string to terminal
				ether.httpPost(PSTR(FEED_PAC), websitepac, PSTR(APIKEY_PAC), str.buf, callback_pac);
				// Wait for reply
				tReply.set(HTTP_TIMEOUT);
				#ifdef UNO
					wdt_disable();
				#endif 
				while (!httpHaveReply) {
					ether.packetLoop(ether.packetReceive());
					if (tReply.poll()) {
						error=1;        // network timeout
						break;
					}
				}
				#ifdef UNO
					wdt_enable(WDTO_8S);
				#endif
				ether.packetLoop(ether.packetReceive());
			}
		#endif
	  
		// Example of posting to emoncms v3 demo account goto http://vis.openenergymonitor.org/emoncms3 
		// and login with sandbox:sandbox
		// To point to your account just enter your WRITE APIKEY 
		request_attempt ++;
		dataReady =0;
	}

	if (request_attempt > 10) {	// Reset the nanode if more than 10 request attempts have been tried without a reply
		ether.begin(sizeof Ethernet::buffer, mymac);
		dhcp_status = 0;
		#ifdef POST2EMONCMS
			dns_status_emon = 0;
		#endif
		#ifdef POST2PACHUBE
			dns_status_pac = 0; 
		#endif
		httpHaveReply = 0;
		request_attempt = 0;
		error=0;
	}
}
//**********************************************************************************************************************

