#include <Fuzzy.h>

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
int lastPosition = 0;

//Neues Fuzzy Objekt erstellen
Fuzzy *fuzzy = new Fuzzy();

//---------------------FUZZY SETS---------------------------



//---------------------Geschwindigkeit----------------------

FuzzySet* Slow = new FuzzySet(0,0,10,20);
FuzzySet* Normal= new FuzzySet(10,20,30,50);
FuzzySet* Fast = new FuzzySet(30,50,130,130);

//----------------------------------------------------------


//----------------------Positon-----------------------------
FuzzySet* FarLeft = new FuzzySet(0,0,40,240); 
FuzzySet* Left = new FuzzySet(0,240,240,620);
FuzzySet* Middle = new FuzzySet(240,620,660,1040);
FuzzySet* Right = new FuzzySet(660,1040,1040,1280);
FuzzySet* FarRight = new FuzzySet(1040,1240,1280,1280);
//----------------------------------------------------------



//--------------------Positionsänderung------------------
FuzzySet* DeadZone = new FuzzySet(0, 0, 7, 50);
FuzzySet* Small = new FuzzySet(7, 50, 150, 250);
FuzzySet* Big = new FuzzySet(150, 250, 1280, 1280);





//--------------------Lenkwinkel---------------------------

FuzzySet* NegativeBig = new FuzzySet(0,0,0,25);
FuzzySet* NegativeSmall = new FuzzySet(0,25,25,45);
FuzzySet* Neutral = new FuzzySet(25,45,45,65);
FuzzySet* PositiveSmall = new  FuzzySet(45,65,65,90);
FuzzySet* PositiveBig = new FuzzySet(65,90,90,90);


//-------------------Gas/Bremse----------------------------

FuzzySet*FullBrake = new FuzzySet (0,0,0,50);
FuzzySet*SlowBrake = new FuzzySet(0,50,50,100);
FuzzySet*Idle = new FuzzySet(50,100,100,150);
FuzzySet*SlowAcceleration = new FuzzySet(100,150,150,200);
FuzzySet*FullAcceleration = new FuzzySet(150,200,200,200);


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

  Speed->addFuzzySet(Slow);
  Speed->addFuzzySet(Normal);
  Speed->addFuzzySet(Fast);
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

//--------------------Positionsänderung--------------------
  FuzzyInput* PositionChange = new FuzzyInput(3);

PositionChange->addFuzzySet(Big);
PositionChange->addFuzzySet(Small);
PositionChange->addFuzzySet(DeadZone);
fuzzy->addFuzzyInput(PositionChange);
//-----------------------Lenkwinkel----------------------------
//                        OUTPUT
  FuzzyOutput *stearingAngle = new FuzzyOutput(1);

  stearingAngle->addFuzzySet(NegativeBig);
  stearingAngle->addFuzzySet(NegativeSmall);
  stearingAngle->addFuzzySet(Neutral);
  stearingAngle->addFuzzySet(PositiveSmall);
  stearingAngle->addFuzzySet(PositiveBig);
  fuzzy->addFuzzyOutput(stearingAngle);


//-------------------------Gas/Bremse----------------------------
//                          OUTPUT
  FuzzyOutput *Acceleration = new FuzzyOutput(2);

  Acceleration->addFuzzySet(FullBrake);
  Acceleration->addFuzzySet(SlowBrake);
  Acceleration->addFuzzySet(Idle);
  Acceleration->addFuzzySet(SlowAcceleration);
  Acceleration->addFuzzySet(FullAcceleration);
  fuzzy->addFuzzyOutput(Acceleration);



//--------------------FUZZY REGELN---------------------------------
FuzzyRuleAntecedent* PositionChangeDeadZone = new FuzzyRuleAntecedent();
PositionChangeDeadZone->joinSingle(DeadZone);

FuzzyRuleAntecedent* PositionChangeSmall = new FuzzyRuleAntecedent();
PositionChangeSmall ->joinSingle(Small);

FuzzyRuleAntecedent* PositionChangeBig = new FuzzyRuleAntecedent();
PositionChangeBig ->joinSingle(Big);




FuzzyRuleAntecedent* SpeedSlowAndPositionFarLeft = new FuzzyRuleAntecedent();
SpeedSlowAndPositionFarLeft ->joinWithAND(Slow, FarLeft);

FuzzyRuleAntecedent* SpeedSlowAndPositionLeft = new FuzzyRuleAntecedent();
SpeedSlowAndPositionLeft ->joinWithAND(Slow, Left);

FuzzyRuleAntecedent* SpeedSlowAndPositionMiddle = new FuzzyRuleAntecedent();
SpeedSlowAndPositionMiddle ->joinWithAND(Slow, Middle);

FuzzyRuleAntecedent* SpeedSlowAndPositionRight = new FuzzyRuleAntecedent();
SpeedSlowAndPositionRight->joinWithAND(Slow, Right);

FuzzyRuleAntecedent* SpeedSlowAndPositionFarRight = new FuzzyRuleAntecedent();
SpeedSlowAndPositionFarRight->joinWithAND(Slow, FarRight);




FuzzyRuleAntecedent* SpeedNormalAndPositionFarLeft= new FuzzyRuleAntecedent();
SpeedNormalAndPositionFarLeft->joinWithAND(Normal, FarLeft);

FuzzyRuleAntecedent* SpeedNormalAndPositionLeft = new FuzzyRuleAntecedent();
SpeedNormalAndPositionLeft->joinWithAND(Normal, Left);

FuzzyRuleAntecedent* SpeedNormalAndPositionMiddle = new FuzzyRuleAntecedent();
SpeedNormalAndPositionMiddle->joinWithAND(Normal, Middle);

FuzzyRuleAntecedent* SpeedNormalAndPositionRight = new FuzzyRuleAntecedent();
SpeedNormalAndPositionRight ->joinWithAND(Normal, Right);

FuzzyRuleAntecedent* SpeedNormalAndPositionFarRight = new FuzzyRuleAntecedent();
SpeedNormalAndPositionFarRight ->joinWithAND(Normal, FarRight);




FuzzyRuleAntecedent* SpeedFastAndPositionFarLeft= new FuzzyRuleAntecedent();
SpeedFastAndPositionFarLeft->joinWithAND(Fast, FarLeft);

FuzzyRuleAntecedent* SpeedFastAndPositionLeft= new FuzzyRuleAntecedent();
SpeedFastAndPositionLeft->joinWithAND(Fast, Left);

FuzzyRuleAntecedent* SpeedFastAndPositionMiddle = new FuzzyRuleAntecedent();
SpeedFastAndPositionMiddle ->joinWithAND(Fast, Middle);

FuzzyRuleAntecedent* SpeedFastAndPositionRight= new FuzzyRuleAntecedent();
SpeedFastAndPositionRight->joinWithAND(Fast, Right);

FuzzyRuleAntecedent* SpeedFastAndPositionFarRight= new FuzzyRuleAntecedent();
SpeedFastAndPositionFarRight->joinWithAND(Fast, FarRight);




FuzzyRuleAntecedent* ifSpeedSlowAndPositionFarLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionFarLeftAndPositionChangeDeadZone->joinWithAND(SpeedSlowAndPositionFarLeft, DeadZone);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionLeftAndPositionChangeDeadZone->joinWithAND(SpeedSlowAndPositionLeft, DeadZone);


FuzzyRuleConsequent* thenStearingAnglePositiveSmallAndAccelerationFullAcceleration = new FuzzyRuleConsequent();
thenStearingAnglePositiveSmallAndAccelerationFullAcceleration->addOutput(PositiveSmall);
thenStearingAnglePositiveSmallAndAccelerationFullAcceleration->addOutput(FullAcceleration);

//Verbundene Regeln

FuzzyRule* FuzzyRule1 = new FuzzyRule(1,ifSpeedSlowAndPositionFarLeftAndPositionChangeDeadZone, thenStearingAnglePositiveSmallAndAccelerationFullAcceleration);
fuzzy->addFuzzyRule(FuzzyRule1);

FuzzyRule* FuzzyRule2 = new FuzzyRule(2, ifSpeedSlowAndPositionLeftAndPositionChangeDeadZone, thenStearingAnglePositiveSmallAndAccelerationFullAcceleration);
fuzzy->addFuzzyRule(FuzzyRule2);




FuzzyRuleAntecedent* ifSpeedSlowAndPositionFarLeftAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionFarLeftAndPositionChangeSmall->joinWithAND(SpeedSlowAndPositionFarLeft, Small);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionLeftAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionLeftAndPositionChangeSmall->joinWithAND(SpeedSlowAndPositionLeft, Small);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionFarLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionFarLeftAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionFarLeft, DeadZone);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionLeftAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionLeft, DeadZone);




FuzzyRuleConsequent* thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration = new FuzzyRuleConsequent();
thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration->addOutput(PositiveSmall);
thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration->addOutput(SlowAcceleration);


//Verbundene Regeln
FuzzyRule* FuzzyRule3 = new FuzzyRule(3, ifSpeedSlowAndPositionFarLeftAndPositionChangeSmall, thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule3);

FuzzyRule* FuzzyRule4 = new FuzzyRule(4, ifSpeedSlowAndPositionLeftAndPositionChangeSmall, thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule4);

FuzzyRule* FuzzyRule5 = new FuzzyRule(5, ifSpeedNormalAndPositionFarLeftAndPositionChangeDeadZone, thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule5);

FuzzyRule* FuzzyRule6 = new FuzzyRule(6, ifSpeedNormalAndPositionLeftAndPositionChangeDeadZone, thenStearingAnglePositiveSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule6);



FuzzyRuleAntecedent* ifSpeedSlowAndPositionFarLeftAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionFarLeftAndPositionChangeBig->joinWithAND(SpeedSlowAndPositionFarLeft,Big);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionLeftAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionLeftAndPositionChangeBig->joinWithAND(SpeedSlowAndPositionLeft, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAnglePositiveBigAndAccelerationSlowAcceleration = new FuzzyRuleConsequent();
thenStearingAnglePositiveBigAndAccelerationSlowAcceleration->addOutput(PositiveBig);
thenStearingAnglePositiveBigAndAccelerationSlowAcceleration->addOutput(SlowAcceleration);


//Verbundene Regeln
FuzzyRule* FuzzyRule7 = new FuzzyRule(7,ifSpeedSlowAndPositionFarLeftAndPositionChangeBig, thenStearingAnglePositiveBigAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule7);

FuzzyRule* FuzzyRule8 = new FuzzyRule(8,ifSpeedSlowAndPositionLeftAndPositionChangeBig, thenStearingAnglePositiveBigAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule8);





FuzzyRuleAntecedent* ifSpeedSlowAndPositionMiddleAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionMiddleAndPositionChangeDeadZone->joinWithAND(SpeedSlowAndPositionMiddle, DeadZone);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionMiddleAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionMiddleAndPositionChangeSmall->joinWithAND(SpeedSlowAndPositionMiddle, Small);

//FuzzyRuleAntecedent* ifSpeedNormalAndPositionLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
//ifSpeedNormalAndPositionLeftAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionLeft, DeadZone);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionMiddleAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionMiddleAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionMiddle, DeadZone);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionMiddleAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionMiddleAndPositionChangeSmall->joinWithAND(SpeedNormalAndPositionMiddle, Small);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionRightAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionRight, DeadZone);


//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNeutralAndAccelerationFullAceleration = new FuzzyRuleConsequent();
thenStearingAngleNeutralAndAccelerationFullAceleration->addOutput(Neutral);
thenStearingAngleNeutralAndAccelerationFullAceleration->addOutput(FullAcceleration);


//Verbundene Regeln
FuzzyRule* FuzzyRule9 = new FuzzyRule(9,ifSpeedSlowAndPositionMiddleAndPositionChangeDeadZone , thenStearingAngleNeutralAndAccelerationFullAceleration);
fuzzy->addFuzzyRule(FuzzyRule9);

FuzzyRule* FuzzyRule10 = new FuzzyRule(10,ifSpeedSlowAndPositionMiddleAndPositionChangeSmall, thenStearingAngleNeutralAndAccelerationFullAceleration);
fuzzy->addFuzzyRule(FuzzyRule10);

FuzzyRule* FuzzyRule11 = new FuzzyRule(11, ifSpeedNormalAndPositionLeftAndPositionChangeDeadZone,thenStearingAngleNeutralAndAccelerationFullAceleration);
fuzzy->addFuzzyRule(FuzzyRule11);

FuzzyRule* FuzzyRule12 = new FuzzyRule(12, ifSpeedNormalAndPositionMiddleAndPositionChangeDeadZone, thenStearingAngleNeutralAndAccelerationFullAceleration);
fuzzy->addFuzzyRule(FuzzyRule12);

FuzzyRule* FuzzyRule13 = new FuzzyRule(13, ifSpeedNormalAndPositionMiddleAndPositionChangeSmall, thenStearingAngleNeutralAndAccelerationFullAceleration);
fuzzy->addFuzzyRule(FuzzyRule13);

FuzzyRule* FuzzyRule14 = new FuzzyRule(14, ifSpeedNormalAndPositionRightAndPositionChangeDeadZone, thenStearingAngleNeutralAndAccelerationFullAceleration);
fuzzy->addFuzzyRule(FuzzyRule14);





FuzzyRuleAntecedent* ifSpeedSlowAndPositionMiddleAndPositionChangeBig  = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionMiddleAndPositionChangeBig->joinWithAND(SpeedSlowAndPositionMiddle, Big);

FuzzyRuleAntecedent* ifSpeedFastAndPostionMiddleAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedFastAndPostionMiddleAndPositionChangeDeadZone->joinWithAND(SpeedFastAndPositionMiddle, DeadZone);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNeutralAndAccelerationSlowAcceleration = new FuzzyRuleConsequent();
thenStearingAngleNeutralAndAccelerationSlowAcceleration->addOutput(Neutral);
thenStearingAngleNeutralAndAccelerationSlowAcceleration->addOutput(SlowAcceleration);


//Verbundene Regeln
FuzzyRule* FuzzyRule15 = new FuzzyRule(15, ifSpeedSlowAndPositionMiddleAndPositionChangeBig, thenStearingAngleNeutralAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule15);

FuzzyRule* FuzzyRule16 = new FuzzyRule(16, ifSpeedFastAndPostionMiddleAndPositionChangeDeadZone, thenStearingAngleNeutralAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule16);





FuzzyRuleAntecedent* ifSpeedSlowAndPositionFarRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionFarRightAndPositionChangeDeadZone->joinWithAND(SpeedSlowAndPositionFarRight, DeadZone);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionRightAndPositionChangeDeadZone->joinWithAND(SpeedSlowAndPositionRight, DeadZone);


//Konsequenzen

FuzzyRuleConsequent* thenStearingAngleNegativeSmallAndAccelerationFullAcceleration = new FuzzyRuleConsequent();
thenStearingAngleNegativeSmallAndAccelerationFullAcceleration->addOutput(NegativeSmall);
thenStearingAngleNegativeSmallAndAccelerationFullAcceleration->addOutput(FullAcceleration);

//Verbundene Regeln

FuzzyRule* FuzzyRule17 = new FuzzyRule(17,ifSpeedSlowAndPositionFarRightAndPositionChangeDeadZone, thenStearingAngleNegativeSmallAndAccelerationFullAcceleration);
fuzzy->addFuzzyRule(FuzzyRule17);

FuzzyRule* FuzzyRule18 = new FuzzyRule(18, ifSpeedSlowAndPositionRightAndPositionChangeDeadZone, thenStearingAngleNegativeSmallAndAccelerationFullAcceleration);
fuzzy->addFuzzyRule(FuzzyRule18);


FuzzyRuleAntecedent* ifSpeedSlowAndPositionFarRightAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionFarRightAndPositionChangeSmall->joinWithAND(SpeedSlowAndPositionFarRight, Small);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionRightAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionRightAndPositionChangeSmall->joinWithAND(SpeedSlowAndPositionRight, Small);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionFarRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionFarRightAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionFarRight, DeadZone);

//FuzzyRuleAntecedent* ifSpeedNormalAndPositionRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
//ifSpeedNormalAndPositionRightAndPositionChangeDeadZone->joinWithAND(SpeedNormalAndPositionRight, DeadZone);

//Consequenzen

FuzzyRuleConsequent* thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration = new FuzzyRuleConsequent();
thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration->addOutput(NegativeSmall);
thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration->addOutput(SlowAcceleration);


//Verbundene Regeln
FuzzyRule* FuzzyRule19 = new FuzzyRule(19, ifSpeedSlowAndPositionFarRightAndPositionChangeSmall, thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule19);

FuzzyRule* FuzzyRule20 = new FuzzyRule(20, ifSpeedSlowAndPositionRightAndPositionChangeSmall, thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule20);

FuzzyRule* FuzzyRule21 = new FuzzyRule(21, ifSpeedNormalAndPositionFarRightAndPositionChangeDeadZone, thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule21);

FuzzyRule* FuzzyRule22 = new FuzzyRule(22, ifSpeedNormalAndPositionRightAndPositionChangeDeadZone, thenStearingAngleNegativeSmallAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule22);




FuzzyRuleAntecedent* ifSpeedSlowAndPositionFarRightAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionFarRightAndPositionChangeBig->joinWithAND(SpeedSlowAndPositionFarRight,Big);

FuzzyRuleAntecedent* ifSpeedSlowAndPositionRightAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedSlowAndPositionRightAndPositionChangeBig->joinWithAND(SpeedSlowAndPositionRight, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNegativeBigAndAccelerationSlowAcceleration = new FuzzyRuleConsequent();
thenStearingAngleNegativeBigAndAccelerationSlowAcceleration->addOutput(NegativeBig);
thenStearingAngleNegativeBigAndAccelerationSlowAcceleration->addOutput(SlowAcceleration);


//Verbundene Regeln
FuzzyRule* FuzzyRule23 = new FuzzyRule(23,ifSpeedSlowAndPositionFarRightAndPositionChangeBig, thenStearingAngleNegativeBigAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule23);

FuzzyRule* FuzzyRule24 = new FuzzyRule(24,ifSpeedSlowAndPositionRightAndPositionChangeBig, thenStearingAngleNegativeBigAndAccelerationSlowAcceleration);
fuzzy->addFuzzyRule(FuzzyRule24);




FuzzyRuleAntecedent* ifSpeedNormalAndPositionFarLeftAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionFarLeftAndPositionChangeSmall->joinWithAND(SpeedNormalAndPositionFarLeft, Small);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionFarLeftAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionFarLeftAndPositionChangeBig->joinWithAND(SpeedNormalAndPositionFarLeft, Big);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionLeftAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionLeftAndPositionChangeBig->joinWithAND(SpeedNormalAndPositionLeft, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAnglePositiveBigAndAccelerationIdle = new FuzzyRuleConsequent();
thenStearingAnglePositiveBigAndAccelerationIdle->addOutput(PositiveBig);
thenStearingAnglePositiveBigAndAccelerationIdle->addOutput(Idle);


//Verbundene Regeln
FuzzyRule* FuzzyRule25 = new FuzzyRule(25,ifSpeedNormalAndPositionFarLeftAndPositionChangeSmall, thenStearingAnglePositiveBigAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule25);

FuzzyRule* FuzzyRule26 = new FuzzyRule(26,ifSpeedNormalAndPositionFarLeftAndPositionChangeBig, thenStearingAnglePositiveBigAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule26);

FuzzyRule* FuzzyRule27 = new FuzzyRule(27,ifSpeedNormalAndPositionLeftAndPositionChangeBig, thenStearingAnglePositiveBigAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule27);




FuzzyRuleAntecedent* ifSpeedNormalAndPositionMiddleAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionMiddleAndPositionChangeBig->joinWithAND(SpeedNormalAndPositionMiddle, Big);

FuzzyRuleAntecedent* ifSpeedFastAndPositionMiddleAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionMiddleAndPositionChangeSmall->joinWithAND(SpeedFastAndPositionMiddle, Small);

FuzzyRuleAntecedent* ifSpeedFastAndPositionMiddleAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionMiddleAndPositionChangeBig->joinWithAND(SpeedFastAndPositionMiddle, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNeutralAndAccelerationIdle = new FuzzyRuleConsequent();
thenStearingAngleNeutralAndAccelerationIdle->addOutput(Neutral);
thenStearingAngleNeutralAndAccelerationIdle->addOutput(Idle);


//Verbundene Regeln
FuzzyRule* FuzzyRule28 = new FuzzyRule(28, ifSpeedNormalAndPositionMiddleAndPositionChangeBig, thenStearingAngleNeutralAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule28);

FuzzyRule* FuzzyRule29 = new FuzzyRule(29, ifSpeedFastAndPositionMiddleAndPositionChangeSmall, thenStearingAngleNeutralAndAccelerationIdle );
fuzzy->addFuzzyRule(FuzzyRule29);

FuzzyRule* FuzzyRule30 = new FuzzyRule(30, ifSpeedFastAndPositionMiddleAndPositionChangeBig, thenStearingAngleNeutralAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule30);





FuzzyRuleAntecedent* ifSpeedNormalAndPositionFarRightAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionFarRightAndPositionChangeSmall->joinWithAND(SpeedNormalAndPositionFarRight, Small);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionFarRightAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionFarRightAndPositionChangeBig->joinWithAND(SpeedNormalAndPositionFarRight, Big);

FuzzyRuleAntecedent* ifSpeedNormalAndPositionRightAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedNormalAndPositionRightAndPositionChangeBig->joinWithAND(SpeedNormalAndPositionRight, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNegativeBigAndAccelerationIdle = new FuzzyRuleConsequent();
thenStearingAngleNegativeBigAndAccelerationIdle->addOutput(NegativeBig);
thenStearingAngleNegativeBigAndAccelerationIdle->addOutput(Idle);


//Verbundene Regeln
FuzzyRule* FuzzyRule31 = new FuzzyRule(31,ifSpeedNormalAndPositionFarRightAndPositionChangeSmall, thenStearingAngleNegativeBigAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule31);

FuzzyRule* FuzzyRule32 = new FuzzyRule(32,ifSpeedNormalAndPositionFarRightAndPositionChangeBig, thenStearingAngleNegativeBigAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule32);

FuzzyRule* FuzzyRule33 = new FuzzyRule(33,ifSpeedNormalAndPositionRightAndPositionChangeBig, thenStearingAngleNegativeBigAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule33);




FuzzyRuleAntecedent* ifSpeedFastAndPositionFarLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionFarLeftAndPositionChangeDeadZone->joinWithAND(SpeedFastAndPositionFarLeft, DeadZone);

FuzzyRuleAntecedent* ifSpeedFastAndPositionFarLeftAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionFarLeftAndPositionChangeSmall->joinWithAND(SpeedFastAndPositionFarLeft, Small);

FuzzyRuleAntecedent* ifSpeedFastAndPositionLeftAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionLeftAndPositionChangeBig->joinWithAND(SpeedFastAndPositionLeft, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAnglePositiveBigAndAccelerationSlowBrake = new FuzzyRuleConsequent();
thenStearingAnglePositiveBigAndAccelerationSlowBrake->addOutput(PositiveBig);
thenStearingAnglePositiveBigAndAccelerationSlowBrake->addOutput(SlowBrake);


//Verbundene Regeln
FuzzyRule* FuzzyRule34 = new FuzzyRule(34, ifSpeedFastAndPositionFarLeftAndPositionChangeDeadZone, thenStearingAnglePositiveBigAndAccelerationSlowBrake);
fuzzy->addFuzzyRule(FuzzyRule34);

FuzzyRule* FuzzyRule35 = new FuzzyRule(35, ifSpeedFastAndPositionFarLeftAndPositionChangeSmall, thenStearingAnglePositiveBigAndAccelerationSlowBrake);
fuzzy->addFuzzyRule(FuzzyRule35);

FuzzyRule* FuzzyRule36 = new FuzzyRule(36, ifSpeedFastAndPositionLeftAndPositionChangeBig, thenStearingAnglePositiveBigAndAccelerationSlowBrake);
fuzzy->addFuzzyRule(FuzzyRule36);




FuzzyRuleAntecedent* ifSpeedFastAndPositionFarRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionFarRightAndPositionChangeDeadZone->joinWithAND(SpeedFastAndPositionFarRight, DeadZone);

FuzzyRuleAntecedent* ifSpeedFastAndPositionFarRightAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionFarRightAndPositionChangeSmall->joinWithAND(SpeedFastAndPositionFarRight, Small);

FuzzyRuleAntecedent* ifSpeedFastAndPositionRightAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionRightAndPositionChangeBig->joinWithAND(SpeedFastAndPositionRight, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNegativeBigAndAccelerationSlowBrake = new FuzzyRuleConsequent();
thenStearingAngleNegativeBigAndAccelerationSlowBrake->addOutput(NegativeBig);
thenStearingAngleNegativeBigAndAccelerationSlowBrake->addOutput(SlowBrake);


//Verbundene Regeln
FuzzyRule* FuzzyRule37 = new FuzzyRule(37, ifSpeedFastAndPositionFarRightAndPositionChangeDeadZone, thenStearingAngleNegativeBigAndAccelerationSlowBrake);
fuzzy->addFuzzyRule(FuzzyRule37);

FuzzyRule* FuzzyRule38 = new FuzzyRule(38, ifSpeedFastAndPositionFarRightAndPositionChangeSmall, thenStearingAngleNegativeBigAndAccelerationSlowBrake);
fuzzy->addFuzzyRule(FuzzyRule38);

FuzzyRule* FuzzyRule39 = new FuzzyRule(39, ifSpeedFastAndPositionRightAndPositionChangeBig, thenStearingAngleNegativeBigAndAccelerationSlowBrake);
fuzzy->addFuzzyRule(FuzzyRule39);





FuzzyRuleAntecedent* ifSpeedFastAndPositionFarRightAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionFarRightAndPositionChangeBig->joinWithAND(SpeedFastAndPositionFarRight, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNegativeBigAndAccelerationFullBrake = new FuzzyRuleConsequent();
thenStearingAngleNegativeBigAndAccelerationFullBrake->addOutput(NegativeBig);
thenStearingAngleNegativeBigAndAccelerationFullBrake->addOutput(FullBrake);


//Verbundene Regeln
FuzzyRule* FuzzyRule40 = new FuzzyRule(40, ifSpeedFastAndPositionFarRightAndPositionChangeBig, thenStearingAngleNegativeBigAndAccelerationFullBrake);
fuzzy->addFuzzyRule(FuzzyRule40);





FuzzyRuleAntecedent* ifSpeedFastAndPositionRightAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionRightAndPositionChangeDeadZone->joinWithAND(SpeedFastAndPositionRight, DeadZone);

FuzzyRuleAntecedent* ifSpeedFastAndPositionRightAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionRightAndPositionChangeSmall->joinWithAND(SpeedFastAndPositionRight, Small);

//Consequenzen
FuzzyRuleConsequent* thenStearingAngleNegativeSmallAndAccelerationIdle = new FuzzyRuleConsequent();
thenStearingAngleNegativeSmallAndAccelerationIdle->addOutput(NegativeSmall);
thenStearingAngleNegativeSmallAndAccelerationIdle->addOutput(Idle);


//Verbundene Regeln
FuzzyRule* FuzzyRule41 = new FuzzyRule(41, ifSpeedFastAndPositionRightAndPositionChangeDeadZone, thenStearingAngleNegativeSmallAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule41);

FuzzyRule* FuzzyRule42 = new FuzzyRule(42, ifSpeedFastAndPositionRightAndPositionChangeSmall, thenStearingAngleNegativeSmallAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule42);






FuzzyRuleAntecedent* ifSpeedFastAndPositionFarLeftAndPositionChangeBig = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionFarLeftAndPositionChangeBig->joinWithAND(SpeedFastAndPositionFarLeft, Big);

//Consequenzen
FuzzyRuleConsequent* thenStearingAnglePositiveBigAndAccelerationFullBrake = new FuzzyRuleConsequent();
thenStearingAnglePositiveBigAndAccelerationFullBrake->addOutput(PositiveBig);
thenStearingAnglePositiveBigAndAccelerationFullBrake->addOutput(FullBrake);


//Verbundene Regeln
FuzzyRule* FuzzyRule43 = new FuzzyRule(43, ifSpeedFastAndPositionFarLeftAndPositionChangeBig, thenStearingAnglePositiveBigAndAccelerationFullBrake);
fuzzy->addFuzzyRule(FuzzyRule43);



FuzzyRuleAntecedent* ifSpeedFastAndPositionLeftAndPositionChangeDeadZone = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionLeftAndPositionChangeDeadZone->joinWithAND(SpeedFastAndPositionLeft, DeadZone);

FuzzyRuleAntecedent* ifSpeedFastAndPositionLeftAndPositionChangeSmall = new FuzzyRuleAntecedent();
ifSpeedFastAndPositionLeftAndPositionChangeSmall->joinWithAND(SpeedFastAndPositionLeft, Small);

//Consequenzen
FuzzyRuleConsequent* thenStearingAnglePositiveSmallAndAccelerationIdle = new FuzzyRuleConsequent();
thenStearingAnglePositiveSmallAndAccelerationIdle->addOutput(PositiveSmall);
thenStearingAnglePositiveSmallAndAccelerationIdle->addOutput(Idle);


//Verbundene Regeln
FuzzyRule* FuzzyRule44 = new FuzzyRule(44, ifSpeedFastAndPositionLeftAndPositionChangeDeadZone, thenStearingAnglePositiveSmallAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule44);

FuzzyRule* FuzzyRule45 = new FuzzyRule(45, ifSpeedFastAndPositionLeftAndPositionChangeSmall, thenStearingAnglePositiveSmallAndAccelerationIdle);
fuzzy->addFuzzyRule(FuzzyRule45);
}

void loop() {
int PositionChange;

  // Warten auf string von VS_Code und umwandeln in Variablen
  String content = "";
  char character;
  int tempoINT;
  int Position;

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


  lcd.clear();
  PositionChange = Position - lastPosition;
  lastPosition = Position;

  
  //FuzzyRegelung Inputs
  fuzzy->setInput(1,tempoINT);
  fuzzy->setInput(2,Position);
  fuzzy->setInput(3,PositionChange);
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