#include <SoftwareSerial.h> 
//Bibliothek ermöglicht die Nutzung digitaler Pins für eine RX/TX 
#include <XInput.h> 
//ermöglicht das Emulieren eines X-Box-Controllers durch einen Arduino mit nativem USB Port

SoftwareSerial myserial(9,10); 
//erschaft einen virtuellen Serialport mit Namen "myserial"
//aus Pin 9 wird der virtuellen RX und aus Pin 10 der virtuellen TX Pin von "myserial"

void setup() {
  myserial.begin(9600);
  //startet die virtuelle Kommunikationsschnittstelle
  XInput.begin();
  //startet das Simulieren des Controllers
  XInput.setJoystickRange(2300,6700);
  XInput.setTriggerRange(0,100);
  //manuelle Begrenzung der Spannweite von Joystick und Triggern
}
void loop() {

  int Positionswert;
  int Beschleunigungswert;
  int Bremse;
  int Gas;
  String readString = "";
  
  while (myserial.available()) {
  //Schleife zum Auslesen des Strings Buchstabe für Buchstabe
     delay(1);
    if (myserial.available() > 0) {
      char c = myserial.read();  
      //liest immer ein Zeichen aus dem serial buffer aus
    if (isControl(c)) {
    //Abbruchkriterium, falls ein Kontrollzeichen in den String gerutscht ist
      break;
    }
      readString += c; 
      //fügt die ausgelesenen Zeichen in einem String zusammen    
    }
 }
 if (readString.length() == 10){ //stellt Vollständigkeit sicher
  String Position = readString.substring(0,5);
  String Beschleunigung = readString.substring(5,10);
  //Zerlegung in Positions- und Beschleunigungswert
  if(Position.length()==5 && Beschleunigung.length()==5){
    int Positionswert = Position.toInt();
    int Beschleunigungswert = Beschleunigung.toInt();
    //Umwandlung der Strings in Integer
    if(Beschleunigungswert<100){
      Bremse = 90-Beschleunigungswert;
      //da ein Beschleunigungswert von 0 bedeutet, dass eine 100prozentige Bremsung erfolgen 
      // soll, diese aber durch einen Triggerinput von 100 realisiert wird, erfolgt an dieser
      // Stelle eine Umrechnung      
      Gas = 0;
      //Trigger zum Gas geben wird auf 0 gesetzt, da ansonsten der Wert aus dem letzten Loop
      // weiter genutzt wird, dann würden Gas und Bremse gleichzeitig gedrückt
      }
    if(Beschleunigungswert>100){
      Gas = Beschleunigungswert-120;
      //da ein Beschleunigungswert von 100 Leerlauf bedeutet, die Spannweite des Triggers 
      // jedoch von 0-100 beträgt, erfolgt auch hier eine Umrechnung
      Bremse = 0;
      //Trigger zum Gas geben wird auf 0 gesetzt, da ansonsten der Wert aus dem letzten Loop
      // weiter genutzt wird, dann würden Gas und Bremse gleichzeitig gedrückt
      }
    XInput.setJoystick(JOY_LEFT, Positionswert, 45);
    //"Positionswert" wird als Wert der x-Achse des linken Joysticks übergeben
    XInput.setTrigger(TRIGGER_RIGHT, Gas);
    XInput.setTrigger(TRIGGER_LEFT, Bremse);
    //Gas und Brems Werte werden dem linken und rechten Trigger übergeben
  }
 }
}