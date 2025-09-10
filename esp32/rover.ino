#include <WiFi.h>
#include <HTTPClient.h>
#include <my_secrets.h>

// const char* SSID = SECRET_SSID;
// const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
const char* SSID = "motorola one 5G ace 1718";
const char* WIFI_PASSWORD = "qwerty45";
const char* serverURL = "http://transcribe.alexdenisova.ru/transcribe";

// HTTP server config:
const String SERVER_NAME = "transcribe.alexdenisova.ru";
const uint16_t SERVER_PORT = 80;
const String HTTP_PATH = "/transcribe";

// Buffer for file data (adjust based on your ESP32's available RAM)
const size_t BUFFER_SIZE = 8192;  // 2KB chunks
uint8_t fileBuffer[BUFFER_SIZE];

const String FORM_BOUNDARY = "boundary";  // the multipart/form-data boundary

void setup() {
  Serial.setRxBufferSize(20000);
  Serial.begin(250000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WIFI_PASSWORD);

  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  // WiFiClient client;
  // get(client);
}

void loop() {
  if (Serial.available() > 0) {
    // Read and send the file
    if (readAndSendFile()) {
      Serial.println("File sent successfully!");
    } else {
      Serial.println("File transfer failed!");
    }

    // Clear any remaining serial data
    while (Serial.available()) {
      Serial.read();
    }
  }

  delay(1000);
}

void get(WiFiClient& client) {
  if (client.connect(SERVER_NAME.c_str(), 80, 100000)) {
    if (client.connected()) {
      client.println("GET /health HTTP/1.1");
      client.println("Host: " + SERVER_NAME);
      client.println("");
    }
  }
  unsigned long startTime = millis();
  unsigned long httpResponseTimeOut = 10000;  // 10 seconds timeout
  while ((millis() - startTime < httpResponseTimeOut)) {
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
    }
    delay(10);
  }
  Serial.println("done");
  client.stop();
}


// Expects data stream with format "<file size>\n<file bytes>"
bool readAndSendFile() {
  HTTPClient http;
  WiFiClient client;

  // Receiving file size
  uint32_t file_size = 0;
  while (Serial.available() > 0) {
    char data = Serial.read();
    if (data != '\n') {
      file_size = file_size * 10 + data - '0';
    } else {
      // Finished receiving file size
      break;
    }
  }
  unsigned long start = millis();

  // Connect to server
  client.connect(SERVER_NAME.c_str(), SERVER_PORT);
  if (!client.connected()) {
    Serial.println("Error: Could not connect to server");
    return false;
  }

  // Send HTTP request with headers
  client.println("POST " + HTTP_PATH + " HTTP/1.1");
  client.println("Host: " + SERVER_NAME);
  client.println("Content-Type: multipart/form-data; boundary=" + FORM_BOUNDARY);

  String header = "--" + FORM_BOUNDARY + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"audio.WAV\"\r\nContent-Type: application/octet-stream\r\n\r\n";
  String footer = "\r\n--" + FORM_BOUNDARY + "--\r\n";
  uint32_t contentLength = header.length() + footer.length() + file_size;
  client.println("Content-Length: " + String(contentLength));
  client.println();
  client.print(header);

  // Read from Serial and send in chunks
  unsigned long startTime = millis();
  size_t totalSent = 0;
  while (totalSent < file_size) {
    if (Serial.available() > 0) {
      size_t toRead = min(int(BUFFER_SIZE), int(file_size - totalSent));
      size_t bytesRead = Serial.readBytes(fileBuffer, toRead);

      if (bytesRead > 0) {
        client.write(fileBuffer, bytesRead);
        totalSent += bytesRead;

        // Progress update
        if (totalSent % 5120 == 0) {  // Every 5KB
          float progress = (totalSent * 100.0) / file_size;
          Serial.printf("Uploaded: %d/%d bytes (%.1f%%)\n", totalSent, file_size, progress);
        } else if (totalSent == file_size) {
          Serial.printf("Uploaded: %d/%d bytes (100%%)\n", totalSent, file_size);
          break;
        }
      }
    }
    delay(1);

    // Check for timeout
    if (millis() - startTime > 10000) { // TODO
      Serial.println("Upload timeout");
      http.end();
      return false;
    }
  }

  // Send multipart body footer
  client.print(footer);

  // Wait for response
  unsigned long responseTimeout = millis();
  while (millis() - responseTimeout < 10000) {
    if (client.available()) {
      // Read response
      String response;
      while (client.available()) {
        response += client.readString();
      }

      Serial.println("Server response:");
      Serial.println(response);

      // Check if response contains success code
      bool success = (response.indexOf("200 OK") != -1) || (response.indexOf("201 Created") != -1) || (response.indexOf("HTTP/1.1 2") != -1);

      http.end();
      return success;
    }
    delay(10);
  }

  Serial.println("Timed out waiting for response");
  return false;
}
