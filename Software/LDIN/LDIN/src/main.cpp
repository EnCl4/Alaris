#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "SmartHD3";
const char* password = "Ni01@En03#sM@@!";

#define LED_PIN 8

WebServer server(80);

bool ledState = false;

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name='viewport' content='width=device-width, initial-scale=1'></head>";
  html += "<body><h1>ESP32-C3 Web Service</h1>";
  html += "<p>LED está: ";
  html += (ledState ? "LIGADO" : "DESLIGADO");
  html += "</p>";
  html += "<a href='/led/on'><button>Ligar LED</button></a>";
  html += "<a href='/led/off'><button>Desligar LED</button></a>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleLedOn() {
  ledState = true;
  digitalWrite(LED_PIN, HIGH);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLedOff() {
  ledState = false;
  digitalWrite(LED_PIN, LOW);
  server.sendHeader("Location", "/");
  server.send(303);
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("01");
  WiFi.begin(ssid, password);
Serial.println("02");
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Status: " + String(WiFi.status()));
Serial.println("IP: " + WiFi.localIP().toString());
Serial.println("RSSI: " + String(WiFi.RSSI()));
  Serial.println("\nConectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/led/on", handleLedOn);
  server.on("/led/off", handleLedOff);

  server.begin();
}

void loop() {
  server.handleClient();
}