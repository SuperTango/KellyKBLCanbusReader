/* Define Joystick connection */
#define UP     15
#define DOWN   17
#define LEFT   16
#define RIGHT  19
#define CLICK  18


void setup() {
  
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(LEFT,INPUT);
  pinMode(RIGHT,INPUT);
  pinMode(CLICK,INPUT);
 
  Serial.begin(9600);
  Serial.println("Joystick demo");

}



void loop() {
  
   if (digitalRead(UP) == 0) {
     Serial.println("Up pressed");}
      
   if (digitalRead(DOWN) == 0) {
      Serial.println("Down pressed");}
      
   if (digitalRead(LEFT) == 0) {
       Serial.println("Left pressed");}
   
   if (digitalRead(RIGHT) == 0) {
       Serial.println("Right pressed");}

   if (digitalRead(CLICK) == 0) {
       Serial.println("Click pressed");}      
       delay(100);
}
