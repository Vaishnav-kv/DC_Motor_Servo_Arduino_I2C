void receiveEvent(int howMany) {
  while (Wire.available() > 0) { // loop through all bytes received
    int received_byte;
    Wire.readBytes((uint8_t*)&received_byte, sizeof(received_byte)); // read received byte as integer
    target = received_byte / 100.00; // Convert integer to float angle with two decimal places
  }
}