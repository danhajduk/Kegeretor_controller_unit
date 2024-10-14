#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"
#include "config.h"
#include "mqtt.h"
#include "control.h"
#include "ota.h"
#include <Preferences.h>
#include "utils.h"

// Preferences object
extern Preferences preferences;

// MQTT topic for publishing beer availability
const char* BEER1_TOPIC = DEVICE_NAME "/sensor/beer1";
const char* BEER2_TOPIC = DEVICE_NAME "/sensor/beer2";

// Global variables and client objects for MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Timer variables for periodic publish
unsigned long previousMillis = 0;
const long publishInterval = 60000;  // Publish interval in milliseconds (e.g., 60000ms = 60 seconds)

struct DeviceInfo {
    String identifiers;
    String model;
    String manufacturer;
};

DeviceInfo deviceInfo = {
    .identifiers = DEVICE_IDENTIFIERS,
    .model = DEVICE_MODEL,
    .manufacturer = DEVICE_MANUFACTURER
};

/**
 * @brief Set the beer availability date in preferences and MQTT.
 * 
 * @param beerType The beer type (beer1 or beer2).
 * @param daysFromNow Number of days from today the beer will be available.
 */
void setBeerAvailability(const String& beerType, int daysFromNow) {
    time_t now;
    struct tm timeinfo;
    char dateStr[7];  // MMDDYY format

    time(&now);  // Get the current time
    localtime_r(&now, &timeinfo);

    // Add daysFromNow to the current day
    timeinfo.tm_mday += daysFromNow;
    mktime(&timeinfo);  // Normalize the time structure

    // Format the date as MMDDYY
    strftime(dateStr, sizeof(dateStr), "%m%d%y", &timeinfo);

    // Store in preferences
    preferences.begin("beer_prefs", false);
    preferences.putString(beerType.c_str(), dateStr);
    preferences.end();

    // Publish to MQTT
    String topic = (beerType == "beer1") ? BEER1_TOPIC : BEER2_TOPIC;
    client.publish(topic.c_str(), dateStr, true);

    printToTelnet(beerType + " availability set to: " + String(dateStr));
}

/**
 * @brief Publishes the Home Assistant MQTT Auto-Discovery configuration.
 * 
 * This function sends discovery configuration messages to Home Assistant, enabling
 * automatic discovery and configuration of the device and its sensors (e.g., temperature
 * sensors, door status, fan states, setpoint, and firmware version attributes).
 */
void publishDiscoveryConfig() {
    // Reusable device JSON string
    String deviceData = String("\"device\":{\"identifiers\":[\"") + deviceInfo.identifiers + "\"]," +
                        "\"model\":\"" + deviceInfo.model + "\"," +
                        "\"manufacturer\":\"" + deviceInfo.manufacturer + "\"}";

    // Inside temperature sensor discovery message
    String insideTempConfig = String(
                              "{\"device_class\":\"temperature\","
                              "\"name\":\"") + DEVICE_FRIENDLY_NAME + " Inside Temp" + 
                              "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/insideTemp\","
                              "\"unit_of_measurement\":\"°C\","
                              "\"value_template\":\"{{ value }}\","
                              "\"unique_id\":\"" + DEVICE_NAME + "_inside_temp\","
                              + deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/insideTemp/config", insideTempConfig.c_str(), true);

    // Outside temperature sensor discovery message
    String outsideTempConfig = String(
                               "{\"device_class\":\"temperature\","
                               "\"name\":\"") + DEVICE_FRIENDLY_NAME + " Outside Temp" + 
                               "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/outsideTemp\","
                               "\"unit_of_measurement\":\"°C\","
                               "\"value_template\":\"{{ value }}\","
                               "\"unique_id\":\"" + DEVICE_NAME + "_outside_temp\","
                               + deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/outsideTemp/config", outsideTempConfig.c_str(), true);

    // Cooling state sensor discovery message with static icon
    String coolingStateConfig = String(
                                "{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Cooling State" + 
                                "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/coolingState\","
                                "\"value_template\":\"{{ value }}\","
                                "\"icon\":\"mdi:snowflake\","
                                "\"unique_id\":\"" + DEVICE_NAME + "_cooling_state\","
                                + deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/binary_sensor/" DEVICE_NAME "/coolingState/config", coolingStateConfig.c_str(), true);

    // Door status sensor discovery message
    String doorStateConfig = String(
                            "{\"device_class\":\"door\","
                            "\"name\":\"") + DEVICE_FRIENDLY_NAME + " Door Status" + 
                            "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/doorStatus\","
                            "\"payload_on\":\"ON\","
                            "\"payload_off\":\"OFF\","
                            "\"unique_id\":\"" + DEVICE_NAME + "_door_status\","
                            + deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/binary_sensor/" DEVICE_NAME "/doorStatus/config", doorStateConfig.c_str(), true);

    // Fan status sensor discovery message with static icon
    String fanStateConfig = String(
                          "{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Fan Status" + 
                          "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/fanStatus\","
                          "\"value_template\":\"{{ value }}\","
                          "\"icon\":\"mdi:fan\","
                          "\"unique_id\":\"" + DEVICE_NAME + "_fan_status\","
                          + deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/fanStatus/config", fanStateConfig.c_str(), true);

    // Setpoint sensor discovery message with box mode
    String setpointConfig = String(
                              "{\"device_class\":\"temperature\","
                              "\"name\":\"") + DEVICE_FRIENDLY_NAME + " Set Point" + 
                              "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/setpoint\","
                              "\"command_topic\":\"" + DEVICE_NAME + "/control/setpoint\","
                              "\"unit_of_measurement\":\"°C\","
                              "\"value_template\":\"{{ value }}\","
                              "\"unique_id\":\"" + DEVICE_NAME + "_setpoint\","
                              + deviceData + ","
                              "\"min\": 0,\"max\": 10,\"step\": 0.1,\"mode\":\"box\"}";
    client.publish(DISCOVERY_PREFIX "/number/" DEVICE_NAME "/setpoint/config", setpointConfig.c_str(), true);

    // Average temperature sensor discovery message with static icon
    String avgTempConfig = String(
                          "{\"device_class\":\"temperature\","
                          "\"name\":\"") + DEVICE_FRIENDLY_NAME + " Average Temp" + 
                          "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/avgTemp\","
                          "\"unit_of_measurement\":\"°C\","
                          "\"value_template\":\"{{ value }}\","
                          "\"icon\":\"mdi:thermometer\","
                          "\"unique_id\":\"" + DEVICE_NAME + "_avg_temp\","
                          + deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/avgTemp/config", avgTempConfig.c_str(), true);

    // Firmware version sensor as an attribute of Update Status
    String updateStatusConfig = String(
                               "{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Update Status" + 
                               "\",\"state_topic\":\"" + DEVICE_NAME + "/sensor/updateStatus\","
                               "\"value_template\":\"{{ value }}\","
                               "\"icon\":\"mdi:update\","
                               "\"unique_id\":\"" + DEVICE_NAME + "_update_status\","
                               + deviceData + ","
                               "\"json_attributes_topic\":\"" + DEVICE_NAME + "/sensor/updateStatus/attributes\"}";
    client.publish(DISCOVERY_PREFIX "/sensor/" DEVICE_NAME "/updateStatus/config", updateStatusConfig.c_str(), true);

    // Beer1 button for setting availability
    String beer1ButtonConfig = String(
        "{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Beer1 Set Button" + 
        "\",\"command_topic\":\"" + DEVICE_NAME + "/control/beer1\"," +
        "\"payload_on\":\"SET\",\"unique_id\":\"" + DEVICE_NAME + "_beer1_button\"," +
        deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/button/" DEVICE_NAME "/beer1/config", beer1ButtonConfig.c_str(), true);

    // Beer2 button for setting availability
    String beer2ButtonConfig = String(
        "{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Beer2 Set Button" + 
        "\",\"command_topic\":\"" + DEVICE_NAME + "/control/beer2\"," +
        "\"payload_on\":\"SET\",\"unique_id\":\"" + DEVICE_NAME + "_beer2_button\"," +
        deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/button/" DEVICE_NAME "/beer2/config", beer2ButtonConfig.c_str(), true);
    
    // Update FW button discovery message
    String updateFWButtonConfig = String(
        "{\"name\":\"") + DEVICE_FRIENDLY_NAME + " Update FW Button" + 
        "\",\"command_topic\":\"" + DEVICE_NAME + "/control/updateFW\"," +
        "\"payload_on\":\"UPDATE\",\"unique_id\":\"" + DEVICE_NAME + "_update_fw_button\"," +
        deviceData + "}";
    client.publish(DISCOVERY_PREFIX "/button/" DEVICE_NAME "/updateFW/config", updateFWButtonConfig.c_str(), true);

}

/**
 * @brief Sends sensor data (temperature, cooling state, door status, and fan status) to the MQTT broker.
 * 
 * This function publishes the latest sensor data to the defined MQTT topics, including:
 * - Inside temperature
 * - Outside temperature
 * - Cooling system state (ON/OFF)
 * - Door status (ON/OFF)
 * - Fan status (ON/OFF)
 * - Average temperature
 * - Cooling set point
 * - Firmware version as an attribute of update status
 * 
 * @param insideTemp The current inside temperature
 * @param outsideTemp The current outside temperature
 * @param coolingOn The current state of the cooling system (true = ON, false = OFF)
 * @param firmwareVersion The current firmware version
 * @param newVersion The latest available firmware version for comparison
 */

void sendSensorData(float insideTemp, float outsideTemp, bool coolingOn, String firmwareVersion, String newVersion) {
    // Publish inside temperature
    String insideTempStr = String(insideTemp);
    client.publish((String(DEVICE_NAME) + "/sensor/insideTemp").c_str(), insideTempStr.c_str(), true);

    // Publish outside temperature
    String outsideTempStr = String(outsideTemp);
    client.publish((String(DEVICE_NAME) + "/sensor/outsideTemp").c_str(), outsideTempStr.c_str(), true);

    // Publish cooling state
    String coolingStateStr = coolingOn ? "ON" : "OFF";
    client.publish((String(DEVICE_NAME) + "/sensor/coolingState").c_str(), coolingStateStr.c_str(), true);

    // Publish door status (ON/OFF)
    String doorStateStr = doorOpen ? "ON" : "OFF";
    client.publish((String(DEVICE_NAME) + "/sensor/doorStatus").c_str(), doorStateStr.c_str(), true);

    // Publish fan status
    String fanStateStr = fanOn ? "ON" : "OFF";
    client.publish((String(DEVICE_NAME) + "/sensor/fanStatus").c_str(), fanStateStr.c_str(), true);

    // Publish average temperature
    String avgTempStr = String(calculateAverageTemp());
    client.publish((String(DEVICE_NAME) + "/sensor/avgTemp").c_str(), avgTempStr.c_str(), true);

    // Publish cooling setpoint
    String setpointStr = String(coolingSetPoint);
    client.publish((String(DEVICE_NAME) + "/sensor/setpoint").c_str(), setpointStr.c_str(), true);
    
    // Publish update status (Up to Date/Update Available)
    String updateStatusStr = (newVersion == firmwareVersion) ? "Up to Date" : "Update Available";
    client.publish((String(DEVICE_NAME) + "/sensor/updateStatus").c_str(), updateStatusStr.c_str(), true);

    // Publish firmware version as an attribute
    String updateStatusAttributes = String("{\"fw_version\":\"") + firmwareVersion + "\"}";
    client.publish((String(DEVICE_NAME) + "/sensor/updateStatus/attributes").c_str(), updateStatusAttributes.c_str(), true);
}

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

    // Handle incoming setpoint adjustment
    if (String(topic) == String(DEVICE_NAME) + "/control/setpoint") {
        String setpointStr = "";
        for (int i = 0; i < length; i++) {
            setpointStr += (char)payload[i];
        }
        coolingSetPoint = setpointStr.toFloat();  // Update cooling setpoint
        updateCoolingSetPoint(coolingSetPoint);
        printToTelnet("Setpoint adjusted to: " + String(coolingSetPoint));

        // Publish the updated setpoint
        String setpointUpdate = String(coolingSetPoint);
        client.publish((String(DEVICE_NAME) + "/sensor/setpoint").c_str(), setpointUpdate.c_str(), true);
    }

    if (String(topic) == String(DEVICE_NAME) + "/control/beer1") {
        setBeerAvailability("beer1", 3);
    } else if (String(topic) == String(DEVICE_NAME) + "/control/beer2") {
        setBeerAvailability("beer2", 3);
    }

    if (String(topic) == String(DEVICE_NAME) + "/control/updateFW") {
        printToTelnet("Firmware update requested via MQTT");
        // Trigger OTA firmware update here
        downloadAndUpdate();
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
        printToTelnet("Attempting MQTT connection...");
        
        // Attempt to connect using the defined user credentials
        if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
            printToTelnet("connected");

            // Subscribe to control topics
            client.subscribe(DEVICE_NAME "/control/setpoint");
            client.subscribe(DEVICE_NAME "/control/beer1");
            client.subscribe(DEVICE_NAME "/control/beer2");
            client.subscribe(DEVICE_NAME "/control/updateFW");


            // Publish Home Assistant auto-discovery config
            publishDiscoveryConfig();
        } else {
            printToTelnet("failed, rc= " + String(client.state()) + " try again in 5 seconds");


            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
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

        // Ensure firmwareVersion and newVersion are passed correctly
        sendSensorData(insideTemp, outsideTemp, coolingOn, VERSION, newVersion);  // Pass all required parameters
    }
}
