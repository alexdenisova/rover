#include "WiFi.h"
#include "HTTPClient.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string.h>
#include <my_secrets.h>  // Library with SECRET_SSID, SECRET_WIFI_PASSWORD

TaskHandle_t Task1;
TaskHandle_t Task2;

// Inserting Wifi creds
const char* SSID = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;

const char* HEALTH_URL = "http://transcribe.alexdenisova.net/health";
const char* TRANSCRIBE_URL = "http://transcribe.alexdenisova.net/transcribe";
String serverName = "transcribe.alexdenisova.net";
const IPAddress DNS_SERVER(192, 168, 0, 202);

#define FIFO_SIZE 50000
#define BUFFER_SIZE 2000
QueueHandle_t fifoQueue = xQueueCreate(FIFO_SIZE, sizeof(char));

void setup() {
  Serial.begin(250000);

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

  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    50000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}


uint32_t idx = 0;
bool end = false;
bool reading_file = false;
// String body;
void loop() {
  // while (Serial.available() > 0) {
  //   if (!reading_file) {
  //     String file_size_str = Serial.readStringUntil('\n');
  //     file_size = file_size_str.toInt();
  //     // Serial.printf("file size: %u\n", file_size);
  //     start_post(file_size);
  //     reading_file = true;
  //   } else {
  //     // int bytesRead = Serial.readBytes(data, 48940);
  //     // char c = Serial.read();
  //     // Serial.printf("bytes read: %d\n", bytesRead);
  //     // start_post(48940);
  //     for (int i = 0; i < file_size; i++) {
  //       char c = Serial.read();
  //       client.print(c);
  //       // client.print(data[i]);
  //       // Serial.print(data[i]);
  //     }
  //     String body = end_post();
  //     Serial.println("body: " + body);
  //   }
  //   // send_body(c);
  //   // idx += 1;
  // }
  // Serial.printf("%u\n", idx);
  // if (WiFi.status() == WL_CONNECTED) {
  //   if (!end && Serial.available() > 0) {
  //     if (!reading_file) {
  //       String file_size_str = Serial.readStringUntil('\n');
  //       file_size = file_size_str.toInt();
  //       Serial.printf("file size: %u\n", file_size);
  //       // start_post(file_size);
  //       reading_file = true;
  //     } else {
  //       while (Serial.available() > 0) {
  //         int bytesRead = Serial.readBytes(data, 2447);
  //         // char c = Serial.read();
  //         // send_body(c);
  //         idx+=2447;
  //       }
  //       Serial.printf("%u\n", idx);


  //       // if (idx == file_size) {
  //       //   Serial.printf("%c %u\n", c, idx);
  //       //   // body = end_post();
  //       //   // end = true;

  //       //   // Serial.println();
  //       //   // Serial.println("body: " + body);
  //       // }
  //     }
  //   }
  // } else {
  //   Serial.println("WiFi Disconnected");
  // }

  // delay(10000);
}


//Task1code: writes to fifo
void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for (;;) {
    while (Serial.available() > 0) {
      char data = Serial.read();
      xQueueSend(fifoQueue, &data, portMAX_DELAY);
    }
    vTaskDelay(xDelay);
  }
}

//Task2code: reads from fifo
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;  // Delay for 1ms

  WiFiClient client;
  client.setTimeout(100000);

  char data;
  bool receiving_file_size = true;
  uint32_t file_size = 0;
  uint32_t bytes_sent = 0;
  char buffer[BUFFER_SIZE + 1];
  memset(buffer, '\0', sizeof(buffer));
  uint32_t idx = 0;

  UBaseType_t messagesInQueue;
  bool yes = false;
  for (;;) {
    messagesInQueue = uxQueueMessagesWaiting(fifoQueue);
    if (yes || messagesInQueue > 48900) {
      yes = true;
      while (xQueueReceive(fifoQueue, &data, 0)) {
        if (receiving_file_size) {
          if (data != '\n') {
            file_size = file_size * 10 + data - '0';
            // file_size = 2500;
          } else {
            receiving_file_size = false;
            Serial.printf("Expecting file size: %u\n", file_size);
            while (start_post(client, file_size) != 0) { vTaskDelay(xDelay); }
          }
          continue;
        }
        if (bytes_sent < file_size) {
          if (data == '\0') {
            client.print(buffer);
            client.print(data);
            // Serial.print(buffer);
            // Serial.print(data);
            bytes_sent += strlen(buffer);
            bytes_sent++;
            memset(buffer, '\0', sizeof(buffer));
            idx = 0;
          } else {
            buffer[idx++] = data;
            // bytes_sent++;
            // Serial.printf("\idx: %u\n", idx);
            if ((idx == BUFFER_SIZE) || (bytes_sent + idx == file_size)) {
              client.print(buffer);
              // Serial.print(buffer);
              bytes_sent += strlen(buffer);
              Serial.printf("\nbytes: %u %u\n", bytes_sent, idx);
              memset(buffer, '\0', sizeof(buffer));
              idx = 0;
            }
          }
        }
        if (bytes_sent == file_size) {
          Serial.printf("\FINAL: %u\n", bytes_sent);
          end_post(client);
          receiving_file_size = true;
          file_size = 0;
        }
      }
    }
    // UBaseType_t messagesInQueue = uxQueueMessagesWaiting(fifoQueue);
    // Serial.printf("\noutside: %u %u\n", bytes_sent, messagesInQueue);
    vTaskDelay(xDelay);
  }
}

// Returns 0 if successfully started POST request.
int start_post(WiFiClient& client, uint32_t file_size) {
  String serverName = "transcribe.alexdenisova.net";
  if (client.connect(serverName.c_str(), 80, 100000)) {
    client.println("POST /transcribe HTTP/1.1");
    client.println("Host: " + serverName);

    String boundary = "boundary";
    String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"1.WAV\"\r\nContent-Type: application/octet-stream\r\n\r\n";
    String tail = "\r\n--" + boundary + "--\r\n";
    uint32_t contentLength = head.length() + tail.length() + file_size;

    client.println("Content-Type: multipart/form-data; boundary=" + boundary);
    client.println("Content-Length: " + String(contentLength));
    client.println();
    client.print(head);

    // Serial.println("POST /transcribe HTTP/1.1");
    // Serial.println("Host: " + serverName);
    // Serial.println("Content-Type: multipart/form-data; boundary=" + boundary);
    // Serial.println("Content-Length: " + String(contentLength));
    // Serial.println();
    // Serial.print(head);

  } else {
    Serial.println("Failed connecting to Server");
    return -1;
  }
  return 0;
}

String end_post(WiFiClient& client) {
  String boundary = "boundary";
  String tail = "\r\n--" + boundary + "--\r\n";
  client.print(tail);
  // Serial.print(tail);

  String body;
  unsigned long startTime = millis();
  unsigned long httpResponseTimeOut = 10000;  // 10 seconds timeout
  char previous[4] = { 0 };
  bool reading_body = false;
  uint32_t i = 0;
  while ((millis() - startTime < httpResponseTimeOut)) {
    // if (!client.connected()) {
    //   Serial.println("Server disconnected!");
    //   client.stop();
    //   // Reconnect logic here...
    // }
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
      // if (reading_body) {
      //   body = client.readStringUntil('\n');
      //   break;
      // } else {
      //   char c = client.read();
      //   memmove(previous, previous + 1, 3);
      //   previous[3] = c;
      //   if (strncmp(previous, "\r\n\r\n", 4) == 0) {
      //     reading_body = true;
      //   }
      // }
    }
    delay(10);
  }
  if (body.length() > 0) {
    Serial.printf("Parsed body: %s\n", body);
  } else {
    Serial.println("Could not parse body");
  }
  client.stop();
  return body;
}
