#include <Fuzzy.h>

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
int lastPosition = 0;

//Neues Fuzzy Objekt erstellen
Fuzzy *fuzzy = new Fuzzy();

//---------------------FUZZY SETS---------------------------



//---------------------Geschwindigkeit----------------------

FuzzySet* slow = new FuzzySet(0,0,0,10);
FuzzySet* semiSlow = new FuzzySet(0,15,15,30);
FuzzySet* normal= new FuzzySet(15,30,30,55);
FuzzySet* semiFast = new FuzzySet(30,55,60,80);
FuzzySet* fast = new FuzzySet(60,80,130,130);

//----------------------------------------------------------


//----------------------Abst-----------------------------
FuzzySet* FarLeft = new FuzzySet(0,0,0,160); 
FuzzySet* Left = new FuzzySet(0,160,240,620);
FuzzySet* Middle = new FuzzySet(240,620,660,880);
FuzzySet* Right = new FuzzySet(660,880,1120,1280);
FuzzySet* FarRight = new FuzzySet(1120,1280,1280,1280);
//----------------------------------------------------------



//--------------------Lenkwinkel---------------------------

FuzzySet* negativeBig = new FuzzySet(0,0,0,2500);
FuzzySet* negativeSmall = new FuzzySet(0,2500,2500,4500);
FuzzySet* neutral = new FuzzySet(2500,4500,4500,6500);
FuzzySet* positiveSmall = new  FuzzySet(4500,6500,6500,9000);
FuzzySet* positiveBig = new FuzzySet(6500,9000,9000,9000);




//-------------------Gas/Bremse----------------------------

FuzzySet*fullBrake = new FuzzySet (0,0,0,50);
FuzzySet*slowBrake = new FuzzySet(0,50,50,100);
FuzzySet*idle = new FuzzySet(50,100,100,150);
FuzzySet*slowAcceleration = new FuzzySet(100,150,150,200);
FuzzySet*fullAcceleration = new FuzzySet(150,200,200,200);


// Pin Ansteuerung
int Pin_Value = 0;
int Pin_Number=11;

//Initialisierung:
// Pin Ansteuerung
int Vorwaerts_pin = 3; //gelb
int Rueckwaerts_pin = 5; //gruen
int Lenkung_pin = 6; //rot links oben
double Vorwaerts_value = 0;
double Rueckwaerts_value = 0;
double Lenkung_value = 0;
int p = 750;

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200); // Kommunikation VSCode
  Serial1.begin(9600); // Kommunikation Arduino Micro
  
  lcd.init();
  lcd.backlight();

  // Setze den Pin als Ausgang
  pinMode(Vorwaerts_pin,OUTPUT);
  pinMode(Rueckwaerts_pin,OUTPUT);
  pinMode(Lenkung_pin,OUTPUT);


  pinMode (Pin_Number, OUTPUT);  

  randomSeed(analogRead(0));

  //FuzzyInputs

  //--------------------Geschwindigkeit--------------------
  //                      INPUT
  FuzzyInput *Speed = new FuzzyInput(1);

  Speed->addFuzzySet(slow);
  Speed->addFuzzySet(semiSlow);
  Speed->addFuzzySet(normal);
  Speed->addFuzzySet(semiFast);
  Speed->addFuzzySet(fast);
  fuzzy->addFuzzyInput(Speed);

//----------------------Position---------------------------
//                        INPUT 
  FuzzyInput *position = new FuzzyInput(2);

  position->addFuzzySet(FarLeft);
  position->addFuzzySet(Left);
  position->addFuzzySet(Middle);
  position->addFuzzySet(Right);
  position->addFuzzySet(FarRight);
  fuzzy->addFuzzyInput(position);

//-----------------------Lenkwinkel----------------------------
//                        OUTPUT
  FuzzyOutput *stearingAngle = new FuzzyOutput(1);

  stearingAngle->addFuzzySet(negativeBig);
  stearingAngle->addFuzzySet(negativeSmall);
  stearingAngle->addFuzzySet(neutral);
  stearingAngle->addFuzzySet(positiveSmall);
  stearingAngle->addFuzzySet(positiveBig);
  fuzzy->addFuzzyOutput(stearingAngle);


//-------------------------Gas/Bremse----------------------------
//                          OUTPUT
  FuzzyOutput *Acceleration = new FuzzyOutput(2);

  Acceleration->addFuzzySet(fullBrake);
  Acceleration->addFuzzySet(slowBrake);
  Acceleration->addFuzzySet(idle);
  Acceleration->addFuzzySet(slowAcceleration);
  Acceleration->addFuzzySet(fullAcceleration);
  fuzzy->addFuzzyOutput(Acceleration);



//--------------------FUZZY REGELN---------------------------------




//--------------------REGEL 1--------------------------------------
// Speed Slow und position FarLeft

// PositivBig und idle
//------------------------------------------------------------------

  FuzzyRuleAntecedent* ifSpeedslowAndpositionFarLeft = new FuzzyRuleAntecedent();
  ifSpeedslowAndpositionFarLeft->joinWithAND(slow, FarLeft);

  FuzzyRuleAntecedent* ifSpeedsemiSlowAndpositionFarLeft = new FuzzyRuleAntecedent();
  ifSpeedsemiSlowAndpositionFarLeft->joinWithAND(semiSlow, FarLeft);


  FuzzyRuleConsequent* thenstearingAnglepositiveBigAndAccelerationidle = new FuzzyRuleConsequent();
  thenstearingAnglepositiveBigAndAccelerationidle->addOutput(positiveBig);
  thenstearingAnglepositiveBigAndAccelerationidle->addOutput(idle);

  FuzzyRule* fuzzyRule1 = new FuzzyRule(1,ifSpeedslowAndpositionFarLeft ,thenstearingAnglepositiveBigAndAccelerationidle );
  fuzzy->addFuzzyRule(fuzzyRule1);

  FuzzyRule* fuzzyRule10 = new FuzzyRule(10,ifSpeedsemiSlowAndpositionFarLeft ,thenstearingAnglepositiveBigAndAccelerationidle );
  fuzzy->addFuzzyRule(fuzzyRule10);


//-------------------------------------------------------------------





  
//--------------------REGEL 2--------------------------------------

// SpeedsemiSlow und position Left
// SpeedsemiSlow und position FarLeft



// PositiveSmall und neutal
//-----------------------------------------------------------------

  FuzzyRuleAntecedent* ifSpeedsemiSlowAndpositionLeft = new FuzzyRuleAntecedent();
  ifSpeedsemiSlowAndpositionLeft->joinWithAND(semiSlow, Left);

  

  FuzzyRuleConsequent* thenstearingAnglepositiveSmallAndAccelerationidle = new FuzzyRuleConsequent();
  thenstearingAnglepositiveSmallAndAccelerationidle->addOutput(positiveSmall);
  thenstearingAnglepositiveSmallAndAccelerationidle->addOutput(idle);

  

  FuzzyRule* fuzzyRule9 = new FuzzyRule(9,ifSpeedsemiSlowAndpositionLeft ,thenstearingAnglepositiveSmallAndAccelerationidle );
  fuzzy->addFuzzyRule(fuzzyRule9);



//------------------------------------------------------------------------------------------------------


// Speed Slow und position Left
// PositiveSmall und slowAcceleration
  FuzzyRuleAntecedent* ifSpeedslowAndpositionLeft = new FuzzyRuleAntecedent();
  ifSpeedslowAndpositionLeft->joinWithAND(slow, Left);
  
  FuzzyRuleConsequent* thenstearingAnglepositiveSmallAndAccelerationslowAcceleration = new FuzzyRuleConsequent();
  thenstearingAnglepositiveSmallAndAccelerationslowAcceleration->addOutput(positiveSmall);
  thenstearingAnglepositiveSmallAndAccelerationslowAcceleration->addOutput(slowAcceleration);

  FuzzyRule* fuzzyRule2 = new FuzzyRule(2,ifSpeedslowAndpositionLeft ,thenstearingAnglepositiveSmallAndAccelerationslowAcceleration );
  fuzzy->addFuzzyRule(fuzzyRule2);



//-----------------------------------------------------------------
 //Speed Slow und position Middle
 // neutral und fullacceleration
  
  FuzzyRuleAntecedent* ifSpeedslowAndpositionMiddle = new FuzzyRuleAntecedent();
  ifSpeedslowAndpositionMiddle->joinWithAND(slow, Middle);

  FuzzyRuleConsequent* thenstearingAngleneutralAndAccelerationfullAcceleration = new FuzzyRuleConsequent();
  thenstearingAngleneutralAndAccelerationfullAcceleration->addOutput(neutral);
  thenstearingAngleneutralAndAccelerationfullAcceleration->addOutput(fullAcceleration);


  FuzzyRule* fuzzyRule3 = new FuzzyRule(3,ifSpeedslowAndpositionMiddle ,thenstearingAngleneutralAndAccelerationfullAcceleration );
  fuzzy->addFuzzyRule(fuzzyRule3);

//-----------------------------------------------------------------







  
  //----------------------------------------------------------------- 
  // Speed Slow und position Right
  // negativeSmall und idle
  FuzzyRuleAntecedent* ifSpeedslowAndpositionRight = new FuzzyRuleAntecedent();
  ifSpeedslowAndpositionRight->joinWithAND(slow, Right);

  FuzzyRuleConsequent* thenstearingAnglenegativeSmallAndAccelerationslowAcceleration = new FuzzyRuleConsequent();
  thenstearingAnglenegativeSmallAndAccelerationslowAcceleration->addOutput(negativeSmall);
  thenstearingAnglenegativeSmallAndAccelerationslowAcceleration->addOutput(slowAcceleration);

  FuzzyRule* fuzzyRule4 = new FuzzyRule(4,ifSpeedslowAndpositionRight ,thenstearingAnglenegativeSmallAndAccelerationslowAcceleration );
  fuzzy->addFuzzyRule(fuzzyRule4);











  //-----------------------------------------------------------------
  // Speedsemi Slow und position FarRight
  // negativeBig und idle

  FuzzyRuleAntecedent* ifSpeedsemiSlowAndpositionFarRight = new FuzzyRuleAntecedent();
  ifSpeedsemiSlowAndpositionFarRight->joinWithAND(semiSlow, FarRight);

  FuzzyRuleAntecedent* ifSpeedslowAndpositionFarRight = new FuzzyRuleAntecedent();
  ifSpeedslowAndpositionFarRight->joinWithAND(slow, FarRight);

  FuzzyRuleConsequent* thenstearingAnglenegativeBigAndAccelerationidle = new FuzzyRuleConsequent();
  thenstearingAnglenegativeBigAndAccelerationidle->addOutput(negativeBig);
  thenstearingAnglenegativeBigAndAccelerationidle->addOutput(idle);

  FuzzyRule* fuzzyRule6 = new FuzzyRule(6,ifSpeedsemiSlowAndpositionFarRight ,thenstearingAnglenegativeBigAndAccelerationidle );
  fuzzy->addFuzzyRule(fuzzyRule6);

  FuzzyRule* fuzzyRule5 = new FuzzyRule(5,ifSpeedslowAndpositionFarRight ,thenstearingAnglenegativeBigAndAccelerationidle );
  fuzzy->addFuzzyRule(fuzzyRule5);

















  //-----------------------------------------------------------------
  // SpeedsemiSlow und position Right
  // negativeSmall und idle

  FuzzyRuleAntecedent* ifSpeedsemiSlowAndpositionRight = new FuzzyRuleAntecedent();
  ifSpeedsemiSlowAndpositionRight->joinWithAND(semiSlow, Right);

  FuzzyRuleConsequent* thenstearingAnglenegatvieSmallAndAccelerationidle = new FuzzyRuleConsequent();
  thenstearingAnglenegatvieSmallAndAccelerationidle->addOutput(negativeSmall);
  thenstearingAnglenegatvieSmallAndAccelerationidle->addOutput(idle);


  FuzzyRule* fuzzyRule7 = new FuzzyRule(7,ifSpeedsemiSlowAndpositionRight ,thenstearingAnglenegatvieSmallAndAccelerationidle );
  fuzzy->addFuzzyRule(fuzzyRule7);



















  //-----------------------------------------------------------------
  // SpeedsemiSlow und position Middle
  // neutral und FullAcceleration

  FuzzyRuleAntecedent* ifSpeedsemiSlowAndpositionMiddle = new FuzzyRuleAntecedent();
  ifSpeedsemiSlowAndpositionMiddle->joinWithAND(semiSlow, Middle);

  FuzzyRuleConsequent* thenstearingAngleneutralAndAccelerationneutral = new FuzzyRuleConsequent();
  thenstearingAngleneutralAndAccelerationfullAcceleration->addOutput(neutral);
  thenstearingAngleneutralAndAccelerationfullAcceleration->addOutput(fullAcceleration);


  FuzzyRule* fuzzyRule8 = new FuzzyRule(8,ifSpeedsemiSlowAndpositionMiddle ,thenstearingAngleneutralAndAccelerationfullAcceleration );
  fuzzy->addFuzzyRule(fuzzyRule8);




















  //-----------------------------------------------------------------
  // Speed normal und position FarLeft
  // positiveBig und slowBrake

  FuzzyRuleAntecedent* ifSpeednormalAndpositionFarLeft = new FuzzyRuleAntecedent();
  ifSpeednormalAndpositionFarLeft->joinWithAND(normal, FarLeft);

  FuzzyRuleConsequent* thenstearingAnglepositiveBigAndAccelerationslowBrake = new FuzzyRuleConsequent();
  thenstearingAnglepositiveBigAndAccelerationslowBrake->addOutput(positiveBig);
  thenstearingAnglepositiveBigAndAccelerationslowBrake->addOutput(slowBrake);

  FuzzyRule* fuzzyRule11 = new FuzzyRule(11,ifSpeednormalAndpositionFarLeft , thenstearingAnglepositiveBigAndAccelerationslowBrake);
  fuzzy->addFuzzyRule(fuzzyRule11);
























  //-----------------------------------------------------------------
  // Speednormal und position Left
  // positiveSmall und slowBrake

  FuzzyRuleAntecedent* ifSpeednormalANDpositionLeft = new FuzzyRuleAntecedent();
  ifSpeednormalANDpositionLeft->joinWithAND(normal, Left);

  FuzzyRuleAntecedent* ifSpeedsemiFastANDpositionLeft = new FuzzyRuleAntecedent();
  ifSpeedsemiFastANDpositionLeft->joinWithAND(semiFast, Left);


  FuzzyRuleConsequent* thenstearingAnglepositiveSmallAndAccelerationslowBrake = new FuzzyRuleConsequent();
  thenstearingAnglepositiveSmallAndAccelerationslowBrake->addOutput(positiveSmall);
  thenstearingAnglepositiveSmallAndAccelerationslowBrake->addOutput(slowBrake);

  FuzzyRule* fuzzyRule12 = new FuzzyRule(12,ifSpeednormalANDpositionLeft ,thenstearingAnglepositiveSmallAndAccelerationslowBrake );
  fuzzy->addFuzzyRule(fuzzyRule12);

   FuzzyRule* fuzzyRule19 = new FuzzyRule(19,ifSpeedsemiFastANDpositionLeft , thenstearingAnglepositiveSmallAndAccelerationslowBrake);
  fuzzy->addFuzzyRule(fuzzyRule19);



























  //-----------------------------------------------------------------
  // Speed  normal und position  Middle
  // neutral und slowAcceleration

  FuzzyRuleAntecedent* ifSpeednormalANDpositionMiddle = new FuzzyRuleAntecedent();
  ifSpeednormalANDpositionMiddle->joinWithAND(normal, Middle);

  FuzzyRuleAntecedent* ifSpeedsemiFastANDpositionMiddle = new FuzzyRuleAntecedent();
  ifSpeedsemiFastANDpositionMiddle->joinWithAND(semiFast, Middle);

  FuzzyRuleConsequent* thenstearingAngleneutralAndAccelerationslowAcceleration = new FuzzyRuleConsequent();
  thenstearingAngleneutralAndAccelerationslowAcceleration->addOutput(neutral);
  thenstearingAngleneutralAndAccelerationslowAcceleration->addOutput(slowAcceleration);

  FuzzyRule* fuzzyRule13 = new FuzzyRule(13,ifSpeednormalANDpositionMiddle ,thenstearingAngleneutralAndAccelerationslowAcceleration );
  fuzzy->addFuzzyRule(fuzzyRule13);

  FuzzyRule* fuzzyRule18 = new FuzzyRule(18,ifSpeedsemiFastANDpositionMiddle ,thenstearingAngleneutralAndAccelerationslowAcceleration );
  fuzzy->addFuzzyRule(fuzzyRule18);
































  //-----------------------------------------------------------------
  // Speed  normal und position Right
  // negativeSmall und slowBrake

  FuzzyRuleAntecedent* ifSpeednormalANDpositionRight = new FuzzyRuleAntecedent();
  ifSpeednormalANDpositionRight->joinWithAND(normal, Right);

  FuzzyRuleAntecedent* ifSpeedsemiFastANDpositionRight = new FuzzyRuleAntecedent();
  ifSpeedsemiFastANDpositionRight->joinWithAND(semiFast, Right);

  FuzzyRuleConsequent* thenstearingAnglenegativeSmallAndAccelerationslowBrake = new FuzzyRuleConsequent();
  thenstearingAnglenegativeSmallAndAccelerationslowBrake->addOutput(negativeSmall);
  thenstearingAnglenegativeSmallAndAccelerationslowBrake->addOutput(slowBrake);
  
  FuzzyRule* fuzzyRule14 = new FuzzyRule(14,ifSpeednormalANDpositionRight ,thenstearingAnglenegativeSmallAndAccelerationslowBrake );
  fuzzy->addFuzzyRule(fuzzyRule14);

  FuzzyRule* fuzzyRule17 = new FuzzyRule(17,ifSpeedsemiFastANDpositionRight ,thenstearingAnglenegativeSmallAndAccelerationslowBrake );
  fuzzy->addFuzzyRule(fuzzyRule17);



























  //-----------------------------------------------------------------
  // Speed normal und position FarRight
  // positiveBig und slowBrake

  FuzzyRuleAntecedent* ifSpeednormalANDpositionFarRight = new FuzzyRuleAntecedent();
  ifSpeednormalANDpositionFarRight->joinWithAND(normal, FarRight);


  FuzzyRuleConsequent* thenstearingAnglenegativeBigAndAccelerationslowBrake = new FuzzyRuleConsequent();
  thenstearingAnglenegativeBigAndAccelerationslowBrake->addOutput(negativeBig);
  thenstearingAnglenegativeBigAndAccelerationslowBrake->addOutput(slowBrake);


  FuzzyRule* fuzzyRule15 = new FuzzyRule(15,ifSpeednormalANDpositionFarRight ,thenstearingAnglenegativeBigAndAccelerationslowBrake );
  fuzzy->addFuzzyRule(fuzzyRule15);























  //-----------------------------------------------------------------
  //Speed semiFast und position FarRight
  //Speed fast und position FarRight
  //positiveBig und fullBrake

  FuzzyRuleAntecedent* ifSpeedsemiFastANDpositionFarRight = new FuzzyRuleAntecedent();
  ifSpeedsemiFastANDpositionFarRight->joinWithAND(semiFast, FarRight);

   FuzzyRuleAntecedent* ifSpeedfastANDpositionFarRight = new FuzzyRuleAntecedent();
  ifSpeedfastANDpositionFarRight->joinWithAND(fast, FarRight);


  FuzzyRuleConsequent* thenstearingAnglenegativeBigAndAccelerationfullBrake = new FuzzyRuleConsequent();
  thenstearingAnglenegativeBigAndAccelerationfullBrake->addOutput(negativeBig);
  thenstearingAnglenegativeBigAndAccelerationfullBrake->addOutput(fullBrake);

  FuzzyRule* fuzzyRule16 = new FuzzyRule(16,ifSpeedsemiFastANDpositionFarRight ,thenstearingAnglenegativeBigAndAccelerationfullBrake );
  fuzzy->addFuzzyRule(fuzzyRule16);

  FuzzyRule* fuzzyRule21 = new FuzzyRule(21,ifSpeedfastANDpositionFarRight , thenstearingAnglenegativeBigAndAccelerationfullBrake);
  fuzzy->addFuzzyRule(fuzzyRule21);












  //-----------------------------------------------------------------
  //SpeedsemiFast und position Right
  //negative small und fullBrake

  FuzzyRuleAntecedent* ifSpeedfastANDpositionRight = new FuzzyRuleAntecedent();
  ifSpeedfastANDpositionRight->joinWithAND(fast, Right);

  


  FuzzyRuleConsequent* thenstearingAnglenegativeSmallAndAccelerationfullBrake = new FuzzyRuleConsequent();
  thenstearingAnglenegativeSmallAndAccelerationfullBrake->addOutput(negativeSmall);
  thenstearingAnglenegativeSmallAndAccelerationfullBrake->addOutput(fullBrake);


  FuzzyRule* fuzzyRule22 = new FuzzyRule(22,ifSpeedfastANDpositionRight ,thenstearingAnglenegativeSmallAndAccelerationfullBrake );
  fuzzy->addFuzzyRule(fuzzyRule22);

  


///------------------------------------------------------------------------------

  FuzzyRuleAntecedent* ifSpeedfastANDpositionLeft = new FuzzyRuleAntecedent();
  ifSpeedfastANDpositionLeft->joinWithAND(fast, Left);

  FuzzyRuleConsequent* thenstearingAnglepositiveSmallAndAccelerationfullBrake = new FuzzyRuleConsequent();
  thenstearingAnglepositiveSmallAndAccelerationfullBrake->addOutput(positiveSmall);
  thenstearingAnglepositiveSmallAndAccelerationfullBrake->addOutput(fullBrake);

  FuzzyRule* fuzzyRule24 = new FuzzyRule(24,ifSpeedfastANDpositionLeft ,thenstearingAnglepositiveSmallAndAccelerationfullBrake );
  fuzzy->addFuzzyRule(fuzzyRule24);





















  
  //-----------------------------------------------------------------
  //Speed fast und position Middle
  //neutral und slowBrake

  FuzzyRuleAntecedent* ifSpeedfastANDpositionMiddle = new FuzzyRuleAntecedent();
  ifSpeedfastANDpositionMiddle->joinWithAND(fast, Middle);

  FuzzyRuleConsequent* thenstearingAngleneutralAndAccelerationslowBrake = new FuzzyRuleConsequent();
  thenstearingAngleneutralAndAccelerationslowBrake->addOutput(neutral);
  thenstearingAngleneutralAndAccelerationslowBrake->addOutput(slowBrake);


  FuzzyRule* fuzzyRule23 = new FuzzyRule(23,ifSpeedfastANDpositionMiddle ,thenstearingAngleneutralAndAccelerationslowBrake );
  fuzzy->addFuzzyRule(fuzzyRule23);





















  

  //-----------------------------------------------------------------
  //Speed fast und position FarLeft
  //positiveBig und fullBrake

  FuzzyRuleAntecedent* ifSpeedfastANDpositionFarLeft = new FuzzyRuleAntecedent();
  ifSpeedfastANDpositionFarLeft->joinWithAND(fast, FarLeft);

  FuzzyRuleAntecedent* ifSpeedsemiFastANDpositionFarLeft = new FuzzyRuleAntecedent();
  ifSpeedsemiFastANDpositionFarLeft->joinWithAND(semiFast, FarLeft);

  FuzzyRuleConsequent* thenstearingAnglepositiveBigAndAccelerationfullBrake = new FuzzyRuleConsequent();
  thenstearingAnglepositiveBigAndAccelerationfullBrake->addOutput(positiveBig);
  thenstearingAnglepositiveBigAndAccelerationfullBrake->addOutput(fullBrake);


  FuzzyRule* fuzzyRule25 = new FuzzyRule(25,ifSpeedfastANDpositionFarLeft ,thenstearingAnglepositiveBigAndAccelerationfullBrake );
  fuzzy->addFuzzyRule(fuzzyRule25);
  

  FuzzyRule* fuzzyRule20 = new FuzzyRule(20,ifSpeedsemiFastANDpositionFarLeft , thenstearingAnglepositiveBigAndAccelerationfullBrake);
  fuzzy->addFuzzyRule(fuzzyRule20);

  

  
  }

void loop() {

int temp;
int PositionChange;
         
 // Zufallszahlen für den Input


/*  float input1 = random(0, 130);
  float input2 = random(-999, 999);*/


// Input  werte zum Testen
/*
  int tempoINT = 90;
  int distance = p;
  p = p+10;
*/

  // Warten auf string von VS_Code und umwandeln in Variablen
  String content = "";
  char character;
  int tempoINT;
  int Position;
  int abc;
  
  int NewInput;

   while(Serial.available()){
    character = Serial.read();
    content.concat(character);
    delay(10);

    if (content.length() == 7) {
      String geschwindigkeit = content.substring(0,3);
      String difference = content.substring(3,7);
      if(geschwindigkeit.length()==3 && difference.length()==4){
        tempoINT = geschwindigkeit.toInt();
        Position = difference.toInt();
      }
      //lcd.print(Position);
      if(Position>999){
        Position = (Position-1000)*-1;        
      }
        
          content = "";

    }
  } 


  // Umrechnung der Positionswerte
 
  Position = Position + 640;
  /*
  lcd.setCursor(0,0);
  lcd.print(Position);

  //Definition New
  PositionChange = Position - temp;
  lcd.setCursor(7,1);
  lcd.print(PositionChange);
  lcd.setCursor(7,0);
  

  temp = Position+0;
  lcd.print(temp);
  abc = Position - PositionChange;
  lcd.setCursor(3,1);
  lcd.print(abc);*/
  lcd.clear();
  int adc;

  //lcd.setCursor(0,0);
 // lcd.print(Position);
  //lastPosition = 9166;
  
  PositionChange = Position - lastPosition;
  //lcd.setCursor(7,0);
 // lcd.print(PositionChange);
  adc = adc+1;


  

  lastPosition = Position;
  //lcd.setCursor(12,0);
  //lcd.print(lastPosition);


 
  

  NewInput = Position - PositionChange;
  //lcd.setCursor(0,1);
 // lcd.print(NewInput);
 // lcd.setCursor(5,1);
 // lcd.print(Position);

  
  
  
/*
  Serial.println("\n\n\n Eingang: ");
  Serial.print("\t\t\tGeschwindigkeit: ");
  Serial.println(tempoINT);
  Serial.print("\t\t\tposition: ");
  Serial.println(distance);
*/
  //FuzzyRegelung Inputs
  fuzzy->setInput(1,tempoINT);
  fuzzy->setInput(2,NewInput);
  fuzzy->fuzzify();


  // FuzzyRegelung Outputs
  int output_lenkung = fuzzy->defuzzify(1);
  int output_gas = fuzzy->defuzzify(2);
  

   // I2C Display prints

  String GES =  String(tempoINT);
  String POS =  String(Position);

  String LW = String(output_lenkung);
  String GAS = String(output_gas);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("GES " + GES + " POS " + POS);
  lcd.setCursor(0,1);
  lcd.print("L " + LW + " G " + GAS);
/*
  Serial.println("\n\n\n Ausgang: ");
  Serial.print("\t\t\tLenkwinkel: ");
  Pin_Value= (output_gas*0.32);
  analogWrite (Pin_Number, Pin_Value);
  Serial.println(output_lenkung);

  Serial.print("\t\t\tGas: ");
  Serial.println(output_gas);

  //Printen der Werte der  Regelung

  Serial.println("\n\nGeschwindigkeit\n");
  Serial.print("slow: ");
  Serial.println(slow->getPertinence()); 
  Serial.print("semiSlow: ");
  Serial.println(semiSlow->getPertinence());
  Serial.print("normal: ");
  Serial.println(normal->getPertinence());
  Serial.print("semiFast: ");
  Serial.println(semiFast->getPertinence()); 
  Serial.print("fast: ");
  Serial.println(fast->getPertinence());


  Serial.print("\n\nPosition\n");
  Serial.print("FarLeft: ");
  Serial.println(FarLeft->getPertinence());
  Serial.print("Left:  ");
  Serial.println(Left->getPertinence());
  Serial.print("Middle: ");
  Serial.println(Middle->getPertinence());
  Serial.print("Right: ");
  Serial.println(Right->getPertinence());
  Serial.print("FarRight: "); 
  Serial.println(FarRight->getPertinence());

  Serial.println("\n\nLenkwinkel\n");
  Serial.print("negativeBig: ");
  Serial.println(negativeBig->getPertinence());
  Serial.print("negativeSmall: ");
  Serial.println(negativeSmall->getPertinence());
  Serial.print("neutral: ");
  Serial.println(neutral->getPertinence());
  Serial.print("positiveSmall: ");  
  Serial.println(positiveSmall->getPertinence()); 
  Serial.print("positiveBig: ");
  Serial.println(positiveBig->getPertinence());

  Serial.println("\n\nGas/Bremse\n");
  Serial.print("fullBrake: ");
  Serial.println(fullBrake->getPertinence());
  Serial.print("slowBrake: ");
  Serial.println(slowBrake->getPertinence());
  Serial.print("idle: ");
  Serial.println(idle->getPertinence());
  Serial.print("slowAcceleration: ");
  Serial.println(slowAcceleration->getPertinence()); 
  Serial.print("fullAcceleration: ");
  Serial.println(fullAcceleration->getPertinence());
*/
//---------------------------------------------------
       int lenkung_value = output_lenkung;
       int gas_value =output_gas;
                
        String a;
        String b;

        a = String(lenkung_value);
        b = String(gas_value);

        while(a.length()< 5)
        {
          a = "0"+a;
        }
        while(b.length()< 5)
        {
          b = "0"+b;
        }
       String c = a+b;
        delay(12);      
       Serial1.print(c);
}