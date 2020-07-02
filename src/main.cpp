// PubNub MQTT example using ESP32.
#include <WiFi.h>
#include <PubSubClient.h>

#include <Arduino.h>
#include <math.h>
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "Arduino_APDS9960.h"
#include "model.h"

// Connection info.
const char* ssid = "user";
const char* password = "password";
const char* mqttServer = "mqtt.eclipse.org";
const int mqttPort = 1883;
const char* clientID = "MQTTExample";
const char* channelName_banana = "A507/fruit/banana";
const char* channelName_apple = "A507/fruit/apple";
const char* channelName_orange = "A507/fruit/orange";
WiFiClient MQTTclient;
PubSubClient client(MQTTclient);
void callback(char* topic, byte* payload, unsigned int length) {
  String payload_buff;
  for (int i=0;i<length;i++) {
    payload_buff = payload_buff+String((char)payload[i]);
  }
  Serial.println(payload_buff); // Print out messages.
}
long lastReconnectAttempt = 0;
boolean reconnect() {
  if (client.connect(clientID)) {
    //client.subscribe(channelName); // Subscribe to channel.
  }
  return client.connected();
}



// Create a memory pool for the nodes in the network
//constexpr int tensor_pool_size = 2 * 1024;
//uint8_t tensor_pool[tensor_pool_size];

// Define the model to be used
const tflite::Model* tflModel;

// Define the interpreter
tflite::MicroInterpreter* tflInterpreter;

// Input/Output nodes for the network
TfLiteTensor* tflInputTensor;
TfLiteTensor* tflOutputTensor;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize];

// array to map gesture index to a name
const char* CLASSES[] = {
  "Banana", // u8"\U0001F34E", // Apple
  "Apple", // u8"\U0001F34C", // Banana
  "Orange" // u8"\U0001F34A"  // Orange
};

#define NUM_CLASSES (sizeof(CLASSES) / sizeof(CLASSES[0]))

void setup() {
  Serial.begin(9600);
  Serial.println("Attempting to connect...");
  WiFi.begin(ssid,password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");

  }
  client.setServer(mqttServer, mqttPort); // Connect to PubNub.
  client.setCallback(callback);
  lastReconnectAttempt = 0;

	// Load the sample sine model
	Serial.println("Loading Tensorflow model....");
	tflModel = tflite::GetModel(model);
	Serial.println("Model loaded!");

	// Define ops resolver and error reporting
	static tflite::ops::micro::AllOpsResolver resolver;

	static tflite::ErrorReporter* error_reporter;
	static tflite::MicroErrorReporter micro_error;
	error_reporter = &micro_error;

	// Instantiate the interpreter 
	static tflite::MicroInterpreter static_interpreter(
		tflModel, resolver, tensorArena, tensorArenaSize, error_reporter
	);

	tflInterpreter = &static_interpreter;

	// Allocate the the model's tensors in the memory pool that was created.
	Serial.println("Allocating tensors to memory pool");
	if(tflInterpreter->AllocateTensors() != kTfLiteOk) {
		Serial.println("There was an error allocating the memory...ooof");
		return;
	}

	// Define input and output nodes
	tflInputTensor = tflInterpreter->input(0);
	tflOutputTensor = tflInterpreter->output(0);
	Serial.println("Starting inferences... Input a number! ");

    if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
  }

}
void loop() {

  int r, g, b, p, c;
  float sum;

  // check if both color and proximity data is available to sample
  while (!APDS.colorAvailable() || !APDS.proximityAvailable()) {}

  // read the color and proximity sensor
  APDS.readColor(r, g, b, c);
  p = APDS.readProximity();
  sum = r + g + b;

  // check if there's an object close and well illuminated enough
  if (p == 0 && c > 10 && sum > 0) {

    float redRatio = r / sum;
    float greenRatio = g / sum;
    float blueRatio = b / sum;

    // input sensor data to model
    tflInputTensor->data.f[0] = redRatio;
    tflInputTensor->data.f[1] = greenRatio;
    tflInputTensor->data.f[2] = blueRatio;

    // Run inferencing
    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk) {
      Serial.println("Invoke failed!");
      while (1);
      return;
    }

    // Output results
    for (int i = 0; i < NUM_CLASSES; i++) {
      Serial.print(CLASSES[i]);
      Serial.print(" ");
      Serial.print(int(tflOutputTensor->data.f[i] * 100));
      Serial.print("%\n");
    }
    Serial.println();


  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) { // Try to reconnect.
      lastReconnectAttempt = now;
      if (reconnect()) { // Attempt to reconnect.
        lastReconnectAttempt = 0;
      }
    }
  } else { // Connected.
    client.loop();
    char bananaString[8];
    char appleString[8];
    char orangeString[8];
    dtostrf(int(tflOutputTensor->data.f[0] * 100), 1, 2, bananaString);
    dtostrf(int(tflOutputTensor->data.f[1] * 100), 1, 2, appleString);
    dtostrf(int(tflOutputTensor->data.f[2] * 100), 1, 2, orangeString);
    client.publish(channelName_banana,bananaString); // Publish message.
    client.publish(channelName_apple,appleString); // Publish message.
    client.publish(channelName_orange,orangeString); // Publish message.
    delay(1000);
  }
    // Wait for the object to be moved away
    while (!APDS.proximityAvailable() || (APDS.readProximity() == 0)) {}
  }

  
}