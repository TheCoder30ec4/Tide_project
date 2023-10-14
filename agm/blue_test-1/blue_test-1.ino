void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  static int count;
  count ++;
  Serial.print("Hello from me");
  Serial.println(count);
}
