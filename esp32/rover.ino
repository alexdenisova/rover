#include <WiFi.h>
#include <my_secrets.h>

const char* SSID = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
const char* serverURL = "http://transcribe.alexdenisova.ru/transcribe";

// HTTP server config:
const String SERVER_NAME = "transcribe.alexdenisova.ru";
const uint16_t SERVER_PORT = 80;
const String HTTP_PATH = "/transcribe";

// Buffer for file data
const size_t BUFFER_SIZE = 8192;  // 8KB chunks
uint8_t fileBuffer[BUFFER_SIZE];

#define HTTP_TIMEOUT 10000                // 10 second timeout for HTTP request/response
const String FORM_BOUNDARY = "boundary";  // the multipart/form-data boundary

WiFiClient client;

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
  // Expecting data stream with format "<file size>\n<file bytes>"
  if (Serial.available() > 0) {
    uint32_t file_size = readFileSize();
    if ((file_size != 0) && readAndSendFile(file_size)) {
      awaitResponse();
    }

    // Clear RX buffer to wait for new file
    while (Serial.available()) {
      Serial.read();
    }
  }

  delay(1);
}

// Get file size from RX
uint32_t readFileSize() {
  uint32_t file_size = 0;
  while (Serial.available() > 0) {
    char data = Serial.read();
    if (data != '\n') {
      if (data < '0' || '9' < data) {
        Serial.println("Error: Expected format '<file size>\n<file bytes>'");
        return 0;
      }
      file_size = file_size * 10 + data - '0';
    } else {
      // Finished receiving file size
      return file_size;
    }
  }
  return 0;
}


// Reads file from RX and sends it to server
// Returns 'true' if successfully sent HTTP request
bool readAndSendFile(uint32_t file_size) {
  // Connect to server
  client.connect(SERVER_NAME.c_str(), SERVER_PORT);
  if (!client.connected()) {
    // Send connection error to STM32
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

  // Read file bytes from RX and send it to server in chunks
  unsigned long uploadStart = millis();
  size_t totalSent = 0;
  while (totalSent < file_size) {
    if (Serial.available() > 0) {
      size_t toRead = min(int(BUFFER_SIZE), int(file_size - totalSent));
      size_t bytesRead = Serial.readBytes(fileBuffer, toRead);

      if (bytesRead > 0) {
        client.write(fileBuffer, bytesRead);
        totalSent += bytesRead;

        if (totalSent == file_size) {
          break;
        }
      }
    }

    // Check for timeout
    if (millis() - uploadStart > HTTP_TIMEOUT) {
      // Send upload error to STM32
      Serial.println("Error: Upload timeout");
      return false;
    }
    delay(1);
  }

  // Send multipart body footer
  client.print(footer);
  return true;
}

// Waits for response from server
void awaitResponse() {
  int statusCode;
  String responseBody;
  unsigned long responseStart = millis();
  while (millis() - responseStart < HTTP_TIMEOUT) {
    if (parseHttpResponse(client, statusCode, responseBody)) {
      responseBody.trim();
      responseBody.toLowerCase();
      if (statusCode >= 200 && statusCode < 300) {
        // Send response body to STM32
        Serial.println("Success: " + responseBody);
      } else {
        // Send HTTP error to STM32
        Serial.printf("Error: Status: %d, Body: %s\n", statusCode, responseBody.c_str());
      }
      client.stop();
      return;
    }
    delay(10);
  }

  // Send timeout error to STM32
  Serial.println("Error: Timed out waiting for response");
  client.stop();
  return;
}

// Parses the HTTP status code and response body
// Returns 'true' if successfully parsed response
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
