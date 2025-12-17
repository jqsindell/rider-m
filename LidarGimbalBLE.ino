#include <Wire.h>
//#include <LIDARLite.h>
#include "BLEDevice.h"

// ----- RANGE SETTINGS -----
int YAW_MIN = 0;
int YAW_MAX = 180;
int PITCH_MIN = 0;
int PITCH_MAX = 180;

float pitchSpeed = 2.0; // degrees per sample
float yawSpeed = 2.0;   // degrees per sample
int sampleDelayMs = 100; // slower delay for BLE + LiDAR


// ----- OBJECTS -----
//Lidar functionality is commented out since I don't have the scanner to test with
//LIDARLite lidar;

// These UUIDs could be different for different firmware versions of your Rider-M
static BLEUUID serviceUUID("0000fee9-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID("d44bc439-abfd-45a2-b575-925416129600");

// MAC Address of the gimbal
static const char* riderMAddress = "ac:9a:22:19:8b:27";

static BLEClient* pClient;
static BLERemoteCharacteristic* pRemoteCharacteristic;

// --- CRC table (checksum values needed to have Rider-M accept commands) ---
static const uint16_t crcTable[256] = {
  0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548, 53677, 57806, 61935, 4657, 528, 12915, 8786, 21173, 17044, 29431, 25302, 37689, 33560, 45947, 41818, 54205, 50076, 62463, 58334, 9314, 13379, 1056, 5121, 25830, 29895, 17572, 21637, 42346, 46411, 34088, 38153, 58862, 62927, 50604, 54669, 13907, 9842, 5649, 1584, 30423, 26358, 22165, 18100, 46939, 42874, 38681, 34616, 63455, 59390, 55197, 51132, 18628, 22757, 26758, 30887, 2112, 6241, 10242, 14371, 51660, 55789, 59790, 63919, 35144, 39273, 43274, 47403, 23285, 19156, 31415, 27286, 6769, 2640, 14899, 10770, 56317, 52188, 64447, 60318, 39801, 35672, 47931, 43802, 27814, 31879, 19684, 23749, 11298, 15363, 3168, 7233, 60846, 64911, 52716, 56781, 44330, 48395, 36200, 40265, 32407, 28342, 24277, 20212, 15891, 11826, 7761, 3696, 65439, 61374, 57309, 53244, 48923, 44858, 40793, 36728, 37256, 33193, 45514, 41451, 53516, 49453, 61774, 57711, 4224, 161, 12482, 8419, 20484, 16421, 28742, 24679, 33721, 37784, 41979, 46042, 49981, 54044, 58239, 62302, 689, 4752, 8947, 13010, 16949, 21012, 25207, 29270, 46570, 42443, 38312, 34185, 62830, 58703, 54572, 50445, 13538, 9411, 5280, 1153, 29798, 25671, 21540, 17413, 42971, 47098, 34713, 38840, 59231, 63358, 50973, 55100, 9939, 14066, 1681, 5808, 26199, 30326, 17941, 22068, 55628, 51565, 63758, 59695, 39368, 35305, 47498, 43435, 22596, 18533, 30726, 26663, 6336, 2273, 14466, 10403, 52093, 56156, 60223, 64286, 35833, 39896, 43963, 48026, 19061, 23124, 27191, 31254, 2801, 6864, 10931, 14994, 64814, 60687, 56684, 52557, 48554, 44427, 40424, 36297, 31782, 27655, 23652, 19525, 15522, 11395, 7392, 3265, 61215, 65342, 53085, 57212, 44955, 49082, 36825, 40952, 28183, 32310, 20053, 24180, 11923, 16050, 3793, 7920
};

// Fills in the appropriate checksum for a given command
uint16_t calcChecksum(uint8_t *bArr, int len) {
  int i = 0;
  uint16_t i2 = 0;
  while (i < len) {
    int i3 = i2 >> 8;
    i2 = ((i2 << 8) & 0xFFFF) ^ crcTable[(bArr[i] & 0xFF) ^ i3];
    i++;
  }
  return i2;
}

void buildCmd(uint8_t x, uint16_t cmd, uint16_t val, uint8_t *out) {
  uint8_t bArr[5];
  bArr[0] = 6;
  bArr[1] = (cmd >> 8) | x;
  bArr[2] = cmd & 0xFF;
  bArr[3] = (val >> 8) & 0xFF;
  bArr[4] = val & 0xFF;
  uint16_t crc = calcChecksum(bArr, 5);
  out[0] = bArr[0];
  out[1] = bArr[1];
  out[2] = bArr[2];
  out[3] = bArr[3];
  out[4] = bArr[4];
  out[5] = (crc >> 8) & 0xFF;
  out[6] = crc & 0xFF;
}

void sendCmd(uint8_t x, uint16_t cmd, uint16_t val) {
  uint8_t packet[7];
  buildCmd(x, cmd, val, packet);
  if (pRemoteCharacteristic && pClient->isConnected()) {
    pRemoteCharacteristic->writeValue(packet, 7, false);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init LiDAR
  //lidar.begin(0, true);
  //lidar.configure(0);

  // Init BLE
  BLEDevice::init("");
  Serial.println("Connecting to Rider-M...");
  pClient = BLEDevice::createClient();
  BLEAddress riderAddr(riderMAddress);

  if (pClient->connect(riderAddr)) {
    Serial.println("Connected to Rider-M");
    BLERemoteService* pService = pClient->getService(serviceUUID);
    if (pService) {
      pRemoteCharacteristic = pService->getCharacteristic(charUUID);
      if (pRemoteCharacteristic && pRemoteCharacteristic->canWriteNoResponse()) {
        Serial.println("Found control characteristic");
        // I'm not sure these init sequence commands are actually necessary, but it works with them in the code
        sendCmd(1, 2, 0);
        sendCmd(1, 4, 0);
        sendCmd(1, 6, 0);
        Serial.println("Sent init sequence");
        // Force Lock mode at startup - we need to be in lock mode or the gimbal will not pan/tilt
        sendCmd(1, 32807, 1);
        Serial.println("Set Lock mode");
      }
    }
  }
  Serial.println("s to start scan");
  Serial.println("# yaw_deg,pitch_deg,distance_cm");
}

void loop() {
  // Wait for user input before starting scan
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') {   // press 's' in Serial Monitor to start scan
      runScan();
    }
  }
}

/* 
Here is where the commands to pan and tilt are sent, tweaking the min/max and delay values is likely necessary to get optinmal performance
It would be pretty trivial to use the commands below to make the gimbal behave however you want in the loop function above
While I was getting bugs ironed out, I took input from the serial monitor to do a "joystick" control where a/d panned and w/s tilted
*/
void runScan() {
  for (float yawAngle = YAW_MIN; yawAngle <= YAW_MAX; yawAngle += yawSpeed) {
    if (yawAngle < (YAW_MAX/2)) {
      sendCmd(0, 4098, 378);  // pan left
    } else {
      sendCmd(0, 4098, 3717); // pan right
    }

    for (float pitchAngle = PITCH_MIN; pitchAngle <= PITCH_MAX; pitchAngle += pitchSpeed) {
      if (pitchAngle < (PITCH_MAX/2)) {
        sendCmd(0, 4097, 3717); // tilt up
      } else {
        sendCmd(0, 4097, 378);  // tilt down
      }

      delay(sampleDelayMs);

//Since I don't have a sensor on hand, just hardcode a distance value for now
     // int dist = lidar.distance();
      int dist = 100;
      if (dist > 0 && dist < 5000) {
        Serial.print(yawAngle);
        Serial.print(",");
        Serial.print(pitchAngle);
        Serial.print(",");
        Serial.println(dist);
      }
    }
  }

  Serial.println("Scan complete.");
}
