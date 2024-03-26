void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  //  Serial.println(pwmVal);
  if (dir == 1) {
    //    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(in1, pwmVal);

  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    //    digitalWrite(in2, HIGH);
    analogWrite(in2, pwmVal);

  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(LED_PIN, HIGH);
  }
}
