// SunDial_SensorProbe - 2026-09-14
// Bench tool for a replacement outer-counter sensor. Wire the sensor to the
// Nano on the desk: 5V, GND, signal -> A3. Open Serial at 115200 and block /
// unblock the beam. Every 250 ms it prints A3 read two ways:
//   plain  = pinMode INPUT (no pull-up)   -> what the original firmware sees
//   pullup = pinMode INPUT_PULLUP         -> what the break-beam build sees
// Push-pull sensor: both columns agree and flip when you block the beam.
// Open-collector sensor: 'plain' is noisy or stuck, 'pullup' flips cleanly.
// Also shows A0-A2 (pull-up on) for the other three sensors.
void setup (){ Serial.begin (115200); Serial.println ("sensor probe: A3 plain / pullup"); }
void loop (){
  pinMode (A3, INPUT);        delayMicroseconds (200); int plain = digitalRead (A3);
  pinMode (A3, INPUT_PULLUP); delayMicroseconds (200); int pull  = digitalRead (A3);
  pinMode (A0, INPUT_PULLUP); pinMode (A1, INPUT_PULLUP); pinMode (A2, INPUT_PULLUP);
  Serial.print ("A3 plain="); Serial.print (plain);
  Serial.print (" pullup=");   Serial.print (pull);
  Serial.print ("   A0="); Serial.print (digitalRead (A0));
  Serial.print (" A1=");   Serial.print (digitalRead (A1));
  Serial.print (" A2=");   Serial.println (digitalRead (A2));
  delay (250);
}
