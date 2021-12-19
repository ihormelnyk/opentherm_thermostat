#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <OpenTherm.h>

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 4;
const int outPin = 5;

//Data wire is connected to 14 pin on the OpenTherm Shield
#define ONE_WIRE_BUS 14

const char* ssid = "Please specify your WIFI SSID";
const char* password = "Please specify your WIFI password";

ESP8266WebServer server(80);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
OpenTherm ot(inPin, outPin);

float sp = 23, //set point
pv = 0, //current temperature
pv_last = 0, //prior temperature
ierr = 0, //integral error
dt = 0, //time between measurements
op = 0; //PID controller output
unsigned long ts = 0, new_ts = 0; //timestamp

const char HTTP_HTML[] PROGMEM = "<!DOCTYPE html>\
<html>\
<head>\
	<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
	<script>\
		window.setInterval(\"update()\", 2000);\
		function update(){\
			var xhr=new XMLHttpRequest();\
			xhr.open(\"GET\", \"/temp\", true);\
			xhr.onreadystatechange = function () {\
				if (xhr.readyState != XMLHttpRequest.DONE || xhr.status != 200) return;\
				document.getElementById('temp').innerHTML=xhr.responseText;\
			};\
			xhr.send();\
		}\
	</script>\
</head>\
<body style=\"text-align:center\">\
    <h1>OpenTherm Thermostat</h1>\
    <font size=\"7\"><span id=\"temp\">{0}</span>&deg;</font>\
    <p>\
    <form method=\"post\">\
        Set to: <input type=\"text\" name=\"sp\" value=\"{1}\" style=\"width:50px\"><br/><br/>\
        <input type=\"submit\" style=\"width:100px\">\
    <form>\
    </p>\
</body>\
</html>";

void ICACHE_RAM_ATTR handleInterrupt() {
	ot.handleInterrupt();
}

float getTemp() {
	return sensors.getTempCByIndex(0);
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt) {
	float Kc = 10.0; // K / %Heater
	float tauI = 50.0; // sec
	float tauD = 1.0;  // sec
	// PID coefficients
	float KP = Kc;
	float KI = Kc / tauI;
	float KD = Kc*tauD;	
	// upper and lower bounds on heater level
	float ophi = 100;
	float oplo = 0;
	// calculate the error
	float error = sp - pv;
	// calculate the integral error
	ierr = ierr + KI * error * dt;	
	// calculate the measurement derivative
	float dpv = (pv - pv_last) / dt;
	// calculate the PID output
	float P = KP * error; //proportional contribution
	float I = ierr; //integral contribution
	float D = -KD * dpv; //derivative contribution
	float op = P + I + D;
	// implement anti-reset windup
	if ((op < oplo) || (op > ophi)) {
		I = I - KI * error * dt;
		// clip output
		op = max(oplo, min(ophi, op));
	}
	ierr = I;	
	Serial.println("sp="+String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
	return op;
}

void handleRoot() {
	digitalWrite(BUILTIN_LED, 1);

	if (server.method() == HTTP_POST) {
		for (uint8_t i = 0; i<server.args(); i++) {
			if (server.argName(i) == "sp") {
				sp = server.arg(i).toFloat();
			}			
		}
	}

	String page = FPSTR(HTTP_HTML);
	page.replace("{0}", String(getTemp()));
	page.replace("{1}", String((int)sp));	
	server.send(200, "text/html", page);
	digitalWrite(BUILTIN_LED, 0);
}


void handleGetTemp() {
	digitalWrite(BUILTIN_LED, 1);	
	server.send(200, "text/plain", String(getTemp()));
	digitalWrite(BUILTIN_LED, 0);
}

void setup(void) {
	pinMode(BUILTIN_LED, OUTPUT);
	digitalWrite(BUILTIN_LED, 0);
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	Serial.println("");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	if (MDNS.begin("thermostat")) {
		Serial.println("MDNS responder started");
	}

	server.on("/", handleRoot);
	server.on("/temp", handleGetTemp);	

	server.begin();
	Serial.println("HTTP server started");

	//Init DS18B20 sensor
	sensors.begin();
	sensors.requestTemperatures();
	sensors.setWaitForConversion(false); //switch to async mode
	pv, pv_last = sensors.getTempCByIndex(0);
	ts = millis();
	ot.begin(handleInterrupt);
}

void loop(void) {	
	new_ts = millis();
	if (new_ts - ts > 1000) {		
		//Set/Get Boiler Status
		bool enableCentralHeating = true;
		bool enableHotWater = true;
		bool enableCooling = false;
		unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
		OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
		if (responseStatus != OpenThermResponseStatus::SUCCESS) {
			Serial.println("Error: Invalid boiler response " + String(response, HEX));
		}		

		pv = sensors.getTempCByIndex(0);
		dt = (new_ts - ts) / 1000.0;
		ts = new_ts;
		if (responseStatus == OpenThermResponseStatus::SUCCESS) {
			op = pid(sp, pv, pv_last, ierr, dt);
			//Set Boiler Temperature
			ot.setBoilerTemperature(op);
		}
		pv_last = pv;
		sensors.requestTemperatures(); //async temperature request
	}
	server.handleClient(); //handle http requests
}
