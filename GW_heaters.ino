#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <PID_v1.h>
#include "OptaBlue.h"
  
#define PERIODIC_UPDATE_TIME 5000 //tego chyba niema w programie
#define UPDATE_INTERVAL 2000 //tego chyba niema w programie


// Stałe do przeliczania PT100 (Callendar–Van Dusen, uproszczony)
const float a = 0.0039083;
const float b = -0.0000005775;
const float mnDAC = 745;


const unsigned long interval = 800; // 0,8 sekundy
unsigned long previousMillis = 0;
bool ledState = false;


const int H1_POWER = 12500;
const int H2_POWER = 12500;
const int H3_POWER = 25000;

const int MAX_POWER = 50000;
const float hysteresis = 4000;
const float h3OnlyLimit = 20000; // do tej mocy tylko H3 +_ histereza

bool h1State = false;
bool h2State = false;
float hX1power = 0;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);



float r0 = 0;
float r1 =0;
float t0 = 0;
float t1 =0;
float tawg=0;
float ta, tb, tc, td;
const float power60 = 11000;
float PowerCorrection = 0; 
const int sampleCountInStack = 60;


double set_temp;
double set_speed;
double VSD_speed;
double VSD_current;
float set_power_value = 0;
float set_speed_value = 0;
int t4=0;
// Zmienne PID
double Setpoint, Input, Output;
double Kp = 5.0, Ki = 0.2, Kd = 2.0;

// Tworzenie obiektu PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


EthernetServer server(502);
ModbusTCPServer modbusTCPServer;

bool buttonStateON;
bool buttonStateOFF;
bool buttonStateEmergency;
bool Heater1StateOK;
bool Heater2StateOK;
bool buttonStateFaultReset;
bool relayStateVSD;
bool pt100_1_Fault;
bool pt100_2_Fault;
bool pt100_Fault = false;



bool heatersON;

float curr_temp;
float out_power;
float out_power2;
unsigned long lastPrint = 0;
double delta = 8;
double delta2;
double BaseTempPower;
const int pin_H1_StateOK = A0;
const int pin_H2_StateOK = A1;
const int pin_ON = A2;
const int pin_OFF = A3;
const int pin_Fault_reset = A4;
const int pin_Emergency_STOP = A5;
const int pin_VSD_Fault = A6;

const int pin_Q_U2_ON = D0;
const int pin_Q_U4_ON = D1;
const int pin_Fault = D2;
const int pin_Motor_ON = D3;



const int adr_set_temp=16390-1;
const int adr_set_speed=16391-1;
const int adr_curr_speed=16392-1;
const int adr_curr_temp=16393-1;
const int adr_motor_current=16394-1;
const int adr_motor_running=16395-1;
const int adr_motor_fault=16396-1;
const int adr_1pt100_fault=16397-1;
const int adr_2pt100_fault=16398-1;
const int adr_1_heater_fault=16399-1;
const int adr_2_heater_fault=16400-1;
const int adr_out_power=16401-1;
const int adr_heater_running=16401-1;
const int adr_h1State=16402-1;
const int adr_h2State=16403-1;
const int adr_hX1power=16404-1;
const int adr_emergencyStop=16405-1;
const int adr_vsdFault=16406-1;
const int adr_Fault=16407-1;
const int adr_Kp=16408-1;
const int adr_Ki=16409-1;
const int adr_Kd=16410-1;





int Running = 0; 
int Fault = 0; 
int Warning = 0; 




void setup() {
  Serial.begin(9600);
//  while (!Serial);
  Heater1StateOK = 1;
  Heater2StateOK = 1;
  buttonStateEmergency = 1;
  buttonStateFaultReset = 0;
  relayStateVSD = 1;

pt100_1_Fault=false;
pt100_2_Fault=false;


//inputs
  pinMode(BTN_USER, INPUT);
  pinMode(pin_H1_StateOK, INPUT);
  pinMode(pin_H2_StateOK, INPUT);
  pinMode(pin_ON, INPUT);
  pinMode(pin_OFF, INPUT);
  pinMode(pin_Fault_reset, INPUT);
  pinMode(pin_Emergency_STOP, INPUT);
  pinMode(pin_VSD_Fault, INPUT);

  //Outputs
  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  pinMode(pin_Q_U2_ON, OUTPUT);
  pinMode(pin_Q_U4_ON, OUTPUT);
  pinMode(pin_Fault, OUTPUT);
  pinMode(pin_Motor_ON, OUTPUT);

  AnalogExpansion::beginChannelAsRtd(OptaController, 0, 0, false, 0.8);
  AnalogExpansion::beginChannelAsRtd(OptaController, 0, 1, false, 0.8);
  AnalogExpansion::beginChannelAsAdc(OptaController, 0, 2, OA_CURRENT_ADC, false, false, false, 0);  //wejście prądowe
  AnalogExpansion::beginChannelAsAdc(OptaController, 0, 3, OA_CURRENT_ADC, false, false, false, 0);  //wejście prądowe
  

  AnalogExpansion::beginChannelAsDac(OptaController, 0, 4, OA_VOLTAGE_DAC, true,  false, OA_SLEW_RATE_0);  //wyjście analogowe
  AnalogExpansion::beginChannelAsDac(OptaController, 0, 7, OA_VOLTAGE_DAC, true,  false, OA_SLEW_RATE_0);  //wyjście analogowe



 out_power=0;



  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  delay(500);

  Serial.print("Modbus TCP serwer uruchomiony na IP: ");
  Serial.println(Ethernet.localIP());

  server.begin();

  if (!modbusTCPServer.begin()) {
    Serial.println("Błąd uruchomienia Modbus TCP Server");
    while (1);
  }


  modbusTCPServer.configureInputRegisters(16387, 1);
  modbusTCPServer.inputRegisterWrite(16387, 5);   
  modbusTCPServer.configureHoldingRegisters(16385, 35); 
  modbusTCPServer.configureInputRegisters(16383, 35); 

  
  

heatersON=false;


 curr_temp=20;


  //  writeDoubleToHoldingRegisters(16385, wartoscDouble);
 // modbusTCPServer.holdingRegisterWrite(16385, 45);  //ustawienie rejestru 386
  //modbusTCPServer.holdingRegisterWrite(16386, 20);


  set_temp = modbusTCPServer.holdingRegisterRead(adr_set_temp);
Setpoint=set_temp;
  //myPID.SetMode(MANUAL);
myPID.SetMode(AUTOMATIC);  //skasowac docelowo
/////////////////////////////////////////////////////

	  Serial.println("*** Opta Analog RTD example ***");
	 
	  OptaController.begin();
	 
//////////////////////////////////////////////////
  Serial.println("Rejestry gotowe");
}







void loop() {
 
  EthernetClient client = server.available();

  if (client) {
    modbusTCPServer.accept(client);
    while (client.connected()) {
      modbusTCPServer.poll();

      // myPID.SetMode(AUTOMATIC);  //skasowac docelowo

      //te dwa były poniżej po PID
      OptaController.update();
      optaAnalogTask();  //odczyt wejść analogowych



      handleInputs(); //odcyt wejść cyfrowych
      handleStateLogic();
      if (Running ==1) processPID();  //do weryfikacji
      if (Running !=1) out_power=0;
      updateHeaters(out_power);

     
     
     
    if (pt100_1_Fault and !pt100_2_Fault) curr_temp=t1;
    if (!pt100_1_Fault and pt100_2_Fault) curr_temp=t0;   
    if (!pt100_1_Fault and !pt100_2_Fault) curr_temp=(t0+t1)/2;
    if (pt100_1_Fault and pt100_2_Fault)  {
      pt100_Fault = true;
    } else {
       pt100_Fault = false;
    }
       


      //    curr_temp=t0;  //finalnie to dac, a poniższe skasowaś




      if (millis() - lastPrint > 1000) {
        lastPrint = millis();

        modbusData();
        myPID.SetTunings(Kp, Ki, Kd);

        serialPrint();
      } //if millis
    }
    Serial.println("Klient rozłączony.");
  } //if client
}


void modbusData() {

  //odczyt danych z HMI
  set_temp = modbusTCPServer.holdingRegisterRead(adr_set_temp);
  set_speed = modbusTCPServer.holdingRegisterRead(adr_set_speed);

  Kp = modbusTCPServer.holdingRegisterRead(adr_Kp)*0.1;
  Ki = modbusTCPServer.holdingRegisterRead(adr_Ki)*0.1;
  Kd = modbusTCPServer.holdingRegisterRead(adr_Kd)*0.1;




  Setpoint=set_temp;

  //wysłanie danych do HMI
  modbusTCPServer.holdingRegisterWrite(adr_curr_temp, curr_temp);
  modbusTCPServer.holdingRegisterWrite(16385, curr_temp);
  modbusTCPServer.holdingRegisterWrite(adr_out_power, out_power);
  modbusTCPServer.holdingRegisterWrite(adr_curr_speed, VSD_speed);
  modbusTCPServer.holdingRegisterWrite(adr_motor_current, VSD_current);
  modbusTCPServer.holdingRegisterWrite(adr_motor_running, Running);
  modbusTCPServer.holdingRegisterWrite(adr_1_heater_fault,!Heater1StateOK);
  modbusTCPServer.holdingRegisterWrite(adr_2_heater_fault,!Heater2StateOK);
  modbusTCPServer.holdingRegisterWrite(adr_hX1power,hX1power);
  modbusTCPServer.holdingRegisterWrite(adr_emergencyStop,buttonStateEmergency);
  modbusTCPServer.holdingRegisterWrite(adr_vsdFault,!relayStateVSD);
  modbusTCPServer.holdingRegisterWrite(adr_Fault,Fault);
  modbusTCPServer.holdingRegisterWrite(adr_1pt100_fault,pt100_1_Fault);
  modbusTCPServer.holdingRegisterWrite(adr_2pt100_fault,pt100_2_Fault);
}


void serialPrint() {

        Serial.println("----------------------------------------------------------------------------");  

        Serial.print("Nastawa temperatury = ");
        Serial.print(set_temp);

        Serial.print("      Curr_temp = ");
        Serial.print(curr_temp);

        Serial.print("Nastawa prędkości = ");
        Serial.println(set_speed);

        Serial.print("Runniung state = ");
        Serial.print(Running);

        Serial.print("    Fault = ");
        Serial.print(Fault);

        Serial.print("    EMSTOP = ");
        Serial.println(buttonStateEmergency);






        Serial.print("PID  IN = ");
        Serial.print(Input);
        Serial.print("    Out = ");
        Serial.print(Output);
        Serial.print("    Setpoint = ");
        Serial.print(Setpoint);
        Serial.print("   Moc wyjściowa = ");
        Serial.print(out_power);

        Serial.print("   Kp = ");
        Serial.print(Kp);

        Serial.print("   Ki = ");
        Serial.print(Ki);

        Serial.print("   Kd = ");
        Serial.println(Kd);



        Serial.print("h1State = ");
        Serial.print(h1State);


        Serial.print("        h2State = ");
        Serial.print(h2State);

        Serial.print("        ta= ");
        Serial.print(ta);
        Serial.print("        tb= ");
        Serial.print(tb); 
        Serial.print("        tc= ");
        Serial.print(tc); 
        Serial.print("        td= ");
        Serial.println(td); 




        Serial.print("CH0: ");
        Serial.print(r0);
        Serial.print(" Ω, ");
        Serial.print(t0);
        Serial.println(" °C");


        Serial.print("CH1: ");
        Serial.print(r1);
        Serial.print(" Ω, ");
        Serial.print(t1);
        Serial.println(" °C");


        Serial.print("VSD speed: ");
        Serial.print(VSD_speed );
        Serial.print("      VSD_current: ");
        Serial.println(VSD_current);




        Serial.print("delta2: ");
        Serial.print(delta2);
        Serial.print("      BaseTempPower: ");
        Serial.println(BaseTempPower);





}


void handleInputs() {
  buttonStateON = digitalRead(pin_ON);
  buttonStateOFF = digitalRead(pin_OFF);
  buttonStateEmergency = digitalRead(pin_Emergency_STOP);
  buttonStateFaultReset = digitalRead(pin_Fault_reset);
  relayStateVSD = digitalRead(pin_VSD_Fault);
  Heater1StateOK = digitalRead(pin_H1_StateOK);
  Heater2StateOK = digitalRead(pin_H2_StateOK);

  digitalWrite(LED_D0,buttonStateON);
  digitalWrite(LED_D1,buttonStateOFF); 
  digitalWrite(LED_D2,Heater1StateOK);
  digitalWrite(LED_D3,Heater2StateOK);
   
}


void handleStateLogic() {


  if(buttonStateEmergency or !Heater1StateOK or ! Heater2StateOK or !relayStateVSD or pt100_Fault){
    Fault = 1;
  }else {
    if (buttonStateFaultReset) Fault = 0;
  }






 // Running = 1;
  if (buttonStateON && buttonStateOFF && Fault == 0 && (Running == 0 or Running==2)) {
    Running = 1;
    heatersON = true;
    myPID.SetMode(AUTOMATIC);
  } 






  if (!buttonStateOFF && Running == 1) {
    Running = 2; // Cooling down
  }

  if (Fault == 1) {
    if (Running == 1) Running = 2;
    //digitalWrite(pin_Fault, HIGH);



    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ledState = !ledState;  // zmiana stanu diody
      digitalWrite(pin_Fault, ledState ? HIGH : LOW);
      Serial.println(ledState); 
      Serial.println(",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,");
    }
  }

  if (Fault == 0) {
    digitalWrite(pin_Fault, LOW);

  }


  if (Running == 1) {

    digitalWrite(pin_Motor_ON, HIGH);
    heatersON=true;
  }

  if (Running == 2) {
    out_power = 0;
    heatersON = false;
        myPID.SetMode(MANUAL);
    digitalWrite(pin_Motor_ON, HIGH);
    if (t0<=50.0)  Running = 0;
  }

  if (Running == 0) {
    out_power = 0;
    heatersON = false;
    digitalWrite(pin_Motor_ON, LOW);
    myPID.SetMode(MANUAL);
    //dodac zatrzymanie falownika
  }



}


void processPID() {
  if (Running == 1) {
    Input = curr_temp;
    double new_set_temp = modbusTCPServer.holdingRegisterRead(adr_set_temp);
    if (new_set_temp != set_temp) {
      set_temp = new_set_temp;
      Setpoint = set_temp;
    }
    myPID.Compute();
  delta2 = set_temp-curr_temp;


  if (delta2>10)
  {

  out_power = Output*50000/255;

  } else {

  BaseTempPower = delta2-10;
  out_power = (Output*50000/255)+(BaseTempPower*500);

  }

// delta dodatnia to za ciepło



  out_power2=out_power;
  if (delta2<12 and delta2>-12)
  {
    //if (out_power2>(delta60+delta2*1000))   
    BaseTempPower=power60+(set_temp-60)*150;
    out_power=BaseTempPower+delta2*1000+PowerCorrection;
    
  }






 
    //updateHeaters(out_power);
  }
}


void updateHeaters(float requestedPower){
//float requestedPower = power;
    // Decyzje binarne z histerezami


  ta=h3OnlyLimit + hysteresis;
  tb=h3OnlyLimit - hysteresis;
  tc=h3OnlyLimit + H1_POWER + hysteresis;
  td=h3OnlyLimit + H1_POWER - hysteresis;



  if (!h1State && requestedPower > (ta)) {
    h1State = true;



    Serial.println("--------------------------------------------");
    Serial.print("rp: ");
    Serial.print(requestedPower);
    Serial.print("    ta: ");
    Serial.println(ta);

  } else if (h1State && requestedPower < (tb)) {
      h1State = false;
  
      Serial.println("--------------------------------------------");
      Serial.print("rp: ");
      Serial.print(requestedPower);
      Serial.print("    tb: ");
      Serial.println(tb);

  }

t4 = h3OnlyLimit + H1_POWER + hysteresis;

    if (!h2State && requestedPower > (tc)) {
        h2State = true;

      

        Serial.println("--------------------------------------------");
        //Serial.print("h2 state: ");
        //Serial.print(h2State);

        Serial.print("rp: ");
        Serial.print(requestedPower);
        Serial.print("    tc: ");
        Serial.println(tc);


    } else if (h2State && requestedPower < (td)) {
        h2State = false;



        Serial.println("--------------------------------------------");
        Serial.print("rp: ");
        Serial.print(requestedPower);
        Serial.print("    td: ");
        Serial.println(td);







    }

    // Oblicz pozostałą moc dla H3
    int remainingPower = requestedPower;
    if (h1State) remainingPower -= H1_POWER;
    if (h2State) remainingPower -= H2_POWER;

 //  float h3Percentage = constrain((float)remainingPower / H3_POWER, 0.0, 0.99);
   float h3Percentage = constrain((float)remainingPower / H3_POWER, 0.0, 1.0);

    // Sterowanie DAC 0–10V

    hX1power = h3Percentage * H3_POWER / 2;
    set_power_value = h3Percentage * 10.0;


    // Cyfrowe wyjścia
    digitalWrite(pin_Q_U2_ON, h1State ? HIGH : LOW);
    digitalWrite(pin_Q_U4_ON, h2State ? HIGH : LOW);








}


	 




	/* -------------------------------------------------------------------------- */
	void optaAnalogTask() {
	  /* -------------------------------------------------------------------------- */
	 static uint16_t period = 10000;
  static uint16_t pulse = 9999;


    AnalogExpansion aexp = OptaController.getExpansion(0);
    r0 = aexp.getRtd(0);
    r1 = aexp.getRtd(1);

    if (r0 > 0 && r0 < 1000000.0) {
      t0 = (-(1.0 / 100.0) * (50.0 * a - 10 * sqrt(b * r0 + 25.0 * pow(a, 2.0) - 100.0 * b))) / b;
      pt100_1_Fault=false;
    } else {

      pt100_1_Fault=true;
    }

    if (r1 > 0 && r1 < 1000000.0) {
      t1 = (-(1.0 / 100.0) * (50.0 * a - 10 * sqrt(b * r1 + 25.0 * pow(a, 2.0) - 100.0 * b))) / b;
      pt100_2_Fault=false;
    } else {
      pt100_2_Fault=true;
    }

//kanały I1, I2, I3, I4, O1, I5, I6, O2
    VSD_speed = aexp.pinCurrent((uint8_t)2)*5;    //wejście analogowe I3
    VSD_current = aexp.pinCurrent((uint8_t)3)*5;    //wejście analogowe I4



    set_speed_value = set_speed / 10;

    aexp.setDac(4, set_power_value*mnDAC);  //sprawdzić czy działa
	  aexp.setDac(7, set_speed_value*mnDAC);  //sprawdzić czy działa 





    if (h1State) {
      aexp.setPwm(OA_FIRST_PWM_CH, period, pulse);
    } else {
      aexp.setPwm(OA_FIRST_PWM_CH, 0, pulse);

    }

    if (h2State) {
      aexp.setPwm(OA_FIRST_PWM_CH+1, period, pulse);
    } else {
      aexp.setPwm(OA_FIRST_PWM_CH+1, 0, pulse);
      
    }


    //aexp.setPwm(OA_FIRST_PWM_CH+1, 2000, 1999);


	}

