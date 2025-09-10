#include <WiFi.h>
#include <HTTPClient.h>
#include <my_secrets.h>

const char* SSID = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
const char* serverURL = "http://transcribe.alexdenisova.ru/transcribe";

// HTTP server config:
const String SERVER_NAME = "transcribe.alexdenisova.ru";
const uint16_t SERVER_PORT = 80;
const String HTTP_PATH = "/transcribe";

// Buffer for file data (adjust based on your ESP32's available RAM)
const size_t BUFFER_SIZE = 8192;  // 2KB chunks
uint8_t fileBuffer[BUFFER_SIZE];

#define HTTP_TIMEOUT 10000                // 10 second timeout for HTTP request/response
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
}

void loop() {
  if (Serial.available() > 0) {
    // Read and send the file
    readAndSendFile();

    // Clear any remaining serial data
    while (Serial.available()) {
      Serial.read();
    }
  }

  delay(1000);
}

// Expects data stream with format "<file size>\n<file bytes>"
void readAndSendFile() {
  WiFiClient client;

  // Receiving file size
  uint32_t file_size = 0;
  while (Serial.available() > 0) {
    char data = Serial.read();
    if (data != '\n') {
      if (data < '0' || '9' < data) {
        Serial.println("Error: Expected format '<file size>\n<file bytes>'");
        return;
      }
      file_size = file_size * 10 + data - '0';
    } else {
      // Finished receiving file size
      break;
    }
  }

  // Connect to server
  client.connect(SERVER_NAME.c_str(), SERVER_PORT);
  if (!client.connected()) {
    Serial.println("Error: Could not connect to server");
    return;
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
  unsigned long uploadStart = millis();
  size_t totalSent = 0;
  while (totalSent < file_size) {
    if (Serial.available() > 0) {
      size_t toRead = min(int(BUFFER_SIZE), int(file_size - totalSent));
      size_t bytesRead = Serial.readBytes(fileBuffer, toRead);

      if (bytesRead > 0) {
        client.write(fileBuffer, bytesRead);
        totalSent += bytesRead;

        // Progress update
        if (totalSent % 5120 == 0) {  // Every 5KB TODO
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
    if (millis() - uploadStart > HTTP_TIMEOUT) {
      Serial.println("Error: Upload timeout");
      return;
    }
  }

  // Send multipart body footer
  client.print(footer);

  // Wait for response
  int statusCode;
  String responseBody;
  unsigned long responseStart = millis();
  while (millis() - responseStart < HTTP_TIMEOUT) {
    if (parseHttpResponse(client, statusCode, responseBody)) {
      responseBody.trim();
      responseBody.toLowerCase();
      if (statusCode >= 200 && statusCode < 300) {
        // Send response body
        Serial.println("Success: " + responseBody);
      } else {
        // Send HTTP error
        Serial.printf("Error: Status: %d, Body: %s\n", statusCode, responseBody.c_str());
      }
      client.stop();
      return;
    }
    delay(10);
  }

  // Send timeout error
  Serial.println("Error: Timed out waiting for response");
  client.stop();
  return;
}

// Parses the HTTP status code and response body
// Returns true if successfully parsed response
bool parseHttpResponse(WiFiClient& client, int& statusCode, String& responseBody) {
  statusCode = 0;
  responseBody = "";

  if (!client.connected() || !client.available()) {
    return false;
  }

  // Read status line
  String statusLine = client.readStringUntil('\n');
  if (statusLine.length() == 0) {
    return false;
  }

  // Parse status code
  int firstSpace = statusLine.indexOf(' ');
  int secondSpace = statusLine.indexOf(' ', firstSpace + 1);
  if (firstSpace == -1 || secondSpace == -1) {
    return false;
  }

  statusCode = statusLine.substring(firstSpace + 1, secondSpace).toInt();

  // Read headers
  int contentLength = -1;
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) {
      break;  // End of headers
    }

    if (line.startsWith("Content-Length:")) {
      int colonPos = line.indexOf(':');
      contentLength = line.substring(colonPos + 1).toInt();
    }
  }

  // Read body
  if (contentLength == 0) {
    return true;
  } else if (contentLength > 0) {
    // Read exact number of bytes
    char buffer[contentLength + 1];
    int bytesRead = client.readBytes(buffer, contentLength);
    buffer[bytesRead] = '\0';
    responseBody = String(buffer);
  } else {
    // Read until connection closes
    while (client.available()) {
      responseBody += (char)client.read();
    }
  }

  return true;
}
