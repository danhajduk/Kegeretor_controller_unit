#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"
#include "config.h"
#include "mqtt.h"
#include "control.h"


// Global variables and client objects for MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Timer variables for periodic publish
unsigned long previousMillis = 0;
const long publishInterval = 60000;  // Publish interval in milliseconds (e.g., 60000ms = 60 seconds)

float avgTemp = 0.0;
/**
 * @brief Callback function for MQTT messages.
 * 
 * This function is triggered whenever a message is received on a subscribed topic.
 * It processes the incoming message and can be used to handle control commands,
 * such as adjusting the cooling set point.
 * 
 * @param topic The topic that received the message
 * @param payload The payload of the message (actual data)
 * @param length The length of the payload
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Handle incoming setpoint adjustment
    if (String(topic) == String(DEVICE_NAME) + "/control/setpoint") {
        String setpointStr = "";
        for (int i = 0; i < length; i++) {
            setpointStr += (char)payload[i];
        }
        coolingSetPoint = setpointStr.toFloat();  // Update cooling setpoint
        Serial.print("Setpoint adjusted to: ");
        Serial.println(coolingSetPoint);

        // Optionally, publish the updated setpoint
        String setpointUpdate = String(coolingSetPoint);
        client.publish((String(DEVICE_NAME) + "/sensor/setpoint").c_str(), setpointUpdate.c_str(), true);
    }
}

/**
 * @brief Reconnects to the MQTT broker if the connection is lost.
 * 
 * This function continuously attempts to reconnect to the MQTT broker
 * in case the connection is lost. It will retry every 5 seconds.
 */
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        
        // Attempt to connect using the defined user credentials
        if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("connected");

            // Subscribe to control topics
            client.subscribe(DEVICE_NAME "/control/setpoint");

            // Publish Home Assistant auto-discovery config
            publishDiscoveryConfig();
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");

            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

/**
 * @brief Publishes the Home Assistant MQTT Auto-Discovery configuration.
 * 
 * This function sends discovery configuration messages to Home Assistant, enabling
 * automatic discovery and configuration of the device and its sensors (e.g., temperature
 * sensors, door status, fan states, and setpoint).
 */
void publishDiscoveryConfig() {
    // Inside temperature sensor discovery message
    String insideTempConfig = String("{\"device_class\":\"temperature\",\"name\":\"") + DEVICE_FRIENDLY_NAME + " Inside Temp" + 
                              "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/insideTemp\",\"unit_of_measurement\":\"°C\",\"value_template\":\"{{ value }}\"" +
                              ",\"unique_id\":\"" + DEVICE_NAME + "_inside_temp\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                              ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/insideTemp/config", insideTempConfig.c_str(), true);

    // Outside temperature sensor discovery message
    String outsideTempConfig = String("{\"device_class\":\"temperature\",\"name\":\"") + DEVICE_FRIENDLY_NAME + " Outside Temp" + 
                               "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/outsideTemp\",\"unit_of_measurement\":\"°C\",\"value_template\":\"{{ value }}\"" +
                               ",\"unique_id\":\"" + DEVICE_NAME + "_outside_temp\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                               ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/outsideTemp/config", outsideTempConfig.c_str(), true);

    // Cooling state sensor discovery message
    String coolingStateConfig = String("{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Cooling State" + 
                                "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/coolingState\",\"value_template\":\"{{ value }}\"" +
                                ",\"unique_id\":\"" + DEVICE_NAME + "_cooling_state\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                                ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}}";
    client.publish(DISCOVERY_PREFIX "/binary_sensor/" DEVICE_NAME "/coolingState/config", coolingStateConfig.c_str(), true);

    // Door status sensor discovery message
    String doorStateConfig = String("{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Door Status" + 
                            "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/doorStatus\",\"value_template\":\"{{ value }}\"" +
                            ",\"unique_id\":\"" + DEVICE_NAME + "_door_status\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                            ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}}";
    client.publish(DISCOVERY_PREFIX "/binary_sensor/" DEVICE_NAME "/doorStatus/config", doorStateConfig.c_str(), true);

    // Fan status sensor discovery message (internal and external fans combined into one)
    String fanStateConfig = String("{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Fan Status" + 
                           "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/fanStatus\",\"value_template\":\"{{ value }}\"" +
                           ",\"unique_id\":\"" + DEVICE_NAME + "_fan_status\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                           ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}}";
    client.publish(DISCOVERY_PREFIX "/binary_sensor/" DEVICE_NAME "/fanStatus/config", fanStateConfig.c_str(), true);

    // Setpoint sensor discovery message (as a number box)
    String setpointConfig = String("{\"device_class\":\"temperature\",\"name\":\"") + DEVICE_FRIENDLY_NAME + " Set Point" + 
                              "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/setpoint\",\"command_topic\":\"" + DEVICE_NAME + "/control/setpoint" +
                              "\",\"unit_of_measurement\":\"°C\",\"value_template\":\"{{ value }}\"" +
                              ",\"unique_id\":\"" + DEVICE_NAME + "_setpoint\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                              ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}," +
                              "\"min\": 0,\"max\": 10,\"step\": 0.1,\"mode\":\"box\"}";  // Mode set to 'box'
    client.publish(DISCOVERY_PREFIX "/number/" DEVICE_NAME "/setpoint/config", setpointConfig.c_str(), true);

    // Average temperature sensor discovery message
    String avgTempConfig = String("{\"device_class\":\"temperature\",\"name\":\"") + DEVICE_FRIENDLY_NAME + " Average Temp" + 
                          "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/avgTemp\",\"unit_of_measurement\":\"°C\",\"value_template\":\"{{ value }}\"" +
                          ",\"unique_id\":\"" + DEVICE_NAME + "_avg_temp\",\"device\":{\"identifiers\":[\"" + DEVICE_IDENTIFIERS + "\"]" +
                          ",\"model\":\"" + DEVICE_MODEL + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"}}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/avgTemp/config", avgTempConfig.c_str(), true);

}

/**
 * @brief Sends sensor data (temperature, cooling state, door status, and fan status) to the MQTT broker.
 * 
 * This function publishes the latest sensor data to the defined MQTT topics, including:
 * - Inside temperature
 * - Outside temperature
 * - Cooling system state (ON/OFF)
 * - Door status (Open/Closed)
 * - Fan status (ON/OFF)
 * - Average temperature
 * - Cooling set point
 * 
 * @param insideTemp The current inside temperature
 * @param outsideTemp The current outside temperature
 * @param coolingOn The current state of the cooling system (true = ON, false = OFF)
 */
void sendSensorData(float insideTemp, float outsideTemp, bool coolingOn) {
    String avgTemp_srt = String(calculateAverageTemp());
    // Publish inside temperature
    String insideTempStr = String(insideTemp);
    client.publish((String(DEVICE_NAME) + "/sensor/insideTemp").c_str(), insideTempStr.c_str(), true);

    // Publish outside temperature
    String outsideTempStr = String(outsideTemp);
    client.publish((String(DEVICE_NAME) + "/sensor/outsideTemp").c_str(), outsideTempStr.c_str(), true);

    // Publish cooling state
    String coolingStateStr = coolingOn ? "ON" : "OFF";
    client.publish((String(DEVICE_NAME) + "/sensor/coolingState").c_str(), coolingStateStr.c_str(), true);

    // Publish door status (open/closed)
    String doorStateStr = doorOpen ? "Open" : "Closed";
    client.publish((String(DEVICE_NAME) + "/sensor/doorStatus").c_str(), doorStateStr.c_str(), true);

    // Publish fan status (using fanOn)
    String fanStateStr = fanOn ? "ON" : "OFF";
    client.publish((String(DEVICE_NAME) + "/sensor/fanStatus").c_str(), fanStateStr.c_str(), true);

    // Publish average temperature
    String avgTempStr = String(avgTemp);
    client.publish((String(DEVICE_NAME) + "/sensor/avgTemp").c_str(), avgTemp_srt.c_str(), true);

    // Publish cooling setpoint
    String setpointStr = String(coolingSetPoint);
    client.publish((String(DEVICE_NAME) + "/sensor/setpoint").c_str(), setpointStr.c_str(), true);
}

/**
 * @brief Sets up the MQTT connection to the broker.
 * 
 * This function configures the MQTT client with the server settings and the callback
 * function for incoming messages.
 */
void setupMQTT() {
    client.setBufferSize(1024);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);
}

/**
 * @brief Handles the MQTT loop and ensures the connection stays alive.
 * 
 * This function should be called in the main loop to keep the MQTT connection alive,
 * process incoming messages, and automatically reconnect if needed.
 * It also periodically publishes sensor data at regular intervals (e.g., every 60 seconds).
 */
void handleMQTT() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Get the current time
    unsigned long currentMillis = millis();

    // Check if the interval has passed to publish sensor data
    if (currentMillis - previousMillis >= publishInterval) {
        // Update the time of the last publish
        previousMillis = currentMillis;

        // Publish sensor data periodically
        sendSensorData(insideTemp, outsideTemp, coolingOn);  // Make sure you have the actual sensor data
    }
}
