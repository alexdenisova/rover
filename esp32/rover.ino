#include "WiFi.h"
#include "HTTPClient.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string.h>
#include <my_secrets.h>  // Library with SECRET_SSID, SECRET_WIFI_PASSWORD

// Inserting Wifi creds
const char* SSID = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
const IPAddress DNS_SERVER(192, 168, 0, 202);

// HTTP server config:
const String SERVER_NAME = "transcribe.alexdenisova.ru";
const uint16_t SERVER_PORT = 80;
const String HTTP_PATH = "/transcribe";

#define FIFO_SIZE 50000   // Size of the FreeRTOS Queue
#define BUFFER_SIZE 2000  // Size of buffer for the WiFiClient
QueueHandle_t fifoQueue = xQueueCreate(FIFO_SIZE, sizeof(char));
TaskHandle_t Task1;
TaskHandle_t Task2;

#define HTTP_TIMEOUT 10000                // 10 second timeout for HTTP response
const String FORM_BOUNDARY = "boundary";  // the multipart/form-data boundary

void setup() {
  Serial.begin(250000);
  Serial.setRxBufferSize(1024);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  WiFi.setDNS(DNS_SERVER);
  Serial.printf("\nConnected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  // TaskReadSerial executed on core 0
  xTaskCreatePinnedToCore(
    vTaskReadSerial,  /* Task function. */
    "TaskReadSerial", /* name of task. */
    10000,            /* Stack size of task */
    NULL,             /* parameter of the task */
    1,                /* priority of the task */
    &Task1,           /* Task handle to keep track of created task */
    0);               /* pin task to core 0 */
  delay(500);

  // TaskHTTPRequest executed on core 1
  xTaskCreatePinnedToCore(
    vTaskHTTPRequest,  /* Task function. */
    "TaskHTTPRequest", /* name of task. */
    50000,             /* Stack size of task */
    NULL,              /* parameter of the task */
    1,                 /* priority of the task */
    &Task2,            /* Task handle to keep track of created task */
    1);                /* pin task to core 1 */
  delay(500);
}

void loop() {
}


// Read from Serial and write to fifoQueue
void vTaskReadSerial(void* pvParameters) {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for (;;) {
    while (Serial.available() > 0) {
      char data = Serial.read();
      xQueueSend(fifoQueue, &data, portMAX_DELAY);
    }
    vTaskDelay(xDelay);
  }
}

// Read from fifoQueue and send HTTP Request
void vTaskHTTPRequest(void* pvParameters) {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  WiFiClient client;

  char data;
  char buffer[BUFFER_SIZE + 1];
  memset(buffer, '\0', sizeof(buffer));
  uint32_t buffer_idx = 0;

  bool receiving_file_size = true;
  bool started_request = false;
  uint32_t file_size = 0;
  uint32_t bytes_sent = 0;
  for (;;) {
    while (xQueueReceive(fifoQueue, &data, 0)) {
      // Expecting data stream with format "<file size>\n<file bytes>"
      if (receiving_file_size) {
        if (data != '\n') {
          file_size = file_size * 10 + data - '0';
        } else {
          // Finished receiving file size
          receiving_file_size = false;
          // Start POST request (needs file size to determine Content-Length)
          started_request = startPostRequest(client, file_size);
        }
        continue;
      }

      // If startPostRequest failed, then we need to just empty the Queue
      if (!started_request) {
        bytes_sent++;
        // If startPostRequest succeeded, then we read Queue and send file bytes to HTTP server
      } else if (bytes_sent < file_size) {
        // Since '\0' is read as end-of-string by client.print(), it needs to be sent as a seperate char
        if (data == '\0') {
          sendBuffer(client, buffer, sizeof(buffer), buffer_idx, bytes_sent);
          client.print(data);  // Send '\0' byte
          bytes_sent++;
        } else {
          buffer[buffer_idx++] = data;
          // sendBuffer if it is full or if the last byte of the file has been read
          if ((buffer_idx == BUFFER_SIZE) || (bytes_sent + buffer_idx == file_size)) {
            sendBuffer(client, buffer, sizeof(buffer), buffer_idx, bytes_sent);
          }
        }
      }

      if (bytes_sent == file_size) {
        if (started_request) {
          // End the POST request and parse response
          endPostRequest(client);
          awaitResponse(client);
        }
        // Return to initial state
        receiving_file_size = true;
        file_size = 0;
        bytes_sent = 0;
      }
    }
    vTaskDelay(xDelay);
  }
}

// Send the bytes in buffer to server, then clear buffer
void sendBuffer(WiFiClient& client, char buffer[], size_t buffer_size, uint32_t& buffer_idx, uint32_t& bytes_sent) {
  client.print(buffer);
  bytes_sent += strlen(buffer);
  memset(buffer, '\0', buffer_size);
  buffer_idx = 0;
}

// Starts the POST request, sends headers and beginning of multipart/form-data body
// Returns true if successfully started POST request
bool startPostRequest(WiFiClient& client, uint32_t file_size) {
  if (client.connect(SERVER_NAME.c_str(), SERVER_PORT)) {
    if (client.connected()) {
      client.println("POST " + HTTP_PATH + " HTTP/1.1");
      client.println("Host: " + SERVER_NAME);

      String head = "--" + FORM_BOUNDARY + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"audio.WAV\"\r\nContent-Type: application/octet-stream\r\n\r\n";
      String tail = "\r\n--" + FORM_BOUNDARY + "--\r\n";
      uint32_t contentLength = head.length() + tail.length() + file_size;

      client.println("Content-Type: multipart/form-data; boundary=" + FORM_BOUNDARY);
      client.println("Content-Length: " + String(contentLength));
      client.println();
      client.print(head);
      return true;
    }
  }
  Serial.println("Error: Could not connect to server");
  return false;
}

// Sends the end of the multipart/form-data body
void endPostRequest(WiFiClient& client) {
  client.print("\r\n--" + FORM_BOUNDARY + "--\r\n");
  return;
}

// Waits for HTTP response and prints it to Serial
void awaitResponse(WiFiClient& client) {
  int statusCode;
  String responseBody;

  // Wait for response until timeout
  unsigned long startTime = millis();
  while ((millis() - startTime < HTTP_TIMEOUT)) {
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
