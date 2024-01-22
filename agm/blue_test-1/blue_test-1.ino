void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  //to run type the code in here:
  delay(2000);
  static int count;
  count ++;
  Serial.print("Hello from me");
  Serial.println(count);
}
