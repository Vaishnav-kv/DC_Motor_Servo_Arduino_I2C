// Function that executes whenever data is received from master
void receiveEvent(int howMany) {
  byte floatBytes[4];
  while (Wire.available() > 0) {  // Loop through all bytes received
    for (int i = 0; i < 4; i++) {
      floatBytes[i] = Wire.read();  // Read received bytes
    }
  }
  target_theta = bytesToFloat(floatBytes);  // Reconstruct float from bytes
}

// Function to reconstruct a float value from its binary representation using IEEE 754 format
float bytesToFloat(byte* bytes) {
  union {
    float floatVal;
    byte byteVal[4];
  } converter;
  for (int i = 0; i < 4; i++) {
    converter.byteVal[i] = bytes[i];
  }
  return converter.floatVal;
}