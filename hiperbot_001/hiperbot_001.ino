/*

 NANO HIPERBOT
 
 ANALOG PINS
 
 0 GSR / PLANTRONIC
 1 SOLO / HUMIDADE
 2 TEMPERATURA
 3 LDR / LUZ
 
 DIGITAL PINS
 
 0 TX
 1 RX
 2 BARGRAPH CLOCK
 3 BARGRAPH RESET
 8 AUDIO OUT
 9 R
 10 G
 11 B
 
 */

int C = 262, D = 294, E = 330, F = 349, G = 392, A = 440, B = 523, P = 0;

int ode[]   = {
  G,B,D,C,E,G,A,F,E,
  C,D,E,F,G,A,B,D,C,C,D,E,D,C,C,
};
int tempo[] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1.5,1,.5,1,1,
  1,1,1,1,1,1,1,1,2,1,1,1,1.5,1,.5,1,1,
};

int pin_planta = A0;
int pin_solo   = A1;
int pin_temp   = A2;
int pin_luz    = A3;

int pin_bgraphclock = 2;
int pin_bgraphreset = 3;
int pin_audio       = 8;
int pin_cor_r       = 9;
int pin_cor_g       = 10;
int pin_cor_b       = 11;

int val_planta = 0;
int val_solo = 0;
int val_temp = 0;
int val_luz = 0;

boolean is_playing = true;

int music_index = 0;
unsigned long music_millis = 0;
unsigned int music_cycle = 100;

unsigned long pause_millis = 0;
unsigned int pause_cycle = 1000;

void setup() {

  Serial.begin(9600);

  pinMode(pin_bgraphclock, OUTPUT);
  pinMode(pin_bgraphreset, OUTPUT);
  pinMode(pin_audio, OUTPUT);
  pinMode(pin_cor_r, OUTPUT);
  pinMode(pin_cor_g, OUTPUT);
  pinMode(pin_cor_b, OUTPUT);

}

void loop(){

  val_planta = map(analogRead(pin_planta), 0, 650, 0, 1023);
  val_solo = analogRead(pin_solo);
  val_temp = analogRead(pin_temp);
  val_luz = analogRead(pin_luz);

  // BARGRAPH

  int n = val_planta * 10 / 1023;

  for (int i = 0; i < n; i++){
    bgraph_clock();
  }
  bgraph_reset();

  // MUSIC

  if(!is_playing && cycleCheck(&pause_millis, pause_cycle) && val_planta < 500){
    is_playing = true;
    music_index = 0;
    music_millis = millis();
  }

  if(is_playing && cycleCheck(&music_millis, music_cycle)){
    if(music_index < sizeof(ode) / sizeof(int) - 1){
      music_index++;
      if(ode[music_index] != 0){
        tone(pin_audio,ode[music_index]);
      } 
      else {
        noTone(pin_audio);
      }
      music_cycle = tempo[music_index] * (1023-val_luz);
    } 
    else {
      is_playing = false;
      pause_millis = millis();
      noTone(pin_audio);
    }
  }

  RGBroutine();

  Serial.print(val_planta);
  Serial.print(" ");
  Serial.print(val_solo);
  Serial.print(" ");
  Serial.print(val_temp);
  Serial.print(" ");
  Serial.println(val_luz);

}

void bgraph_clock(){
  digitalWrite(pin_bgraphclock, HIGH);
  delay(1);
  digitalWrite(pin_bgraphclock, LOW);
}

void bgraph_reset(){
  digitalWrite(pin_bgraphreset, HIGH);
  delay(1);
  digitalWrite(pin_bgraphreset, LOW);
}

boolean cycleCheck(unsigned long *lastMillis, unsigned int cycle)
{
  unsigned long currentMillis = millis();
  if(currentMillis - *lastMillis >= cycle)
  {
    *lastMillis = currentMillis;
    return true;
  }
  else
    return false;
}


void RGBroutine()
{
  int potVal = val_planta;

  int redVal = 0;
  int grnVal = 0;
  int bluVal = 0;

  if (potVal < 341)
  {
    potVal = (potVal * 3) / 4;
    redVal = 256 - potVal;
    grnVal = potVal;
    bluVal = 1;
  }
  else if (potVal < 682)
  {
    potVal = ((potVal - 341) * 3) / 4;
    redVal = 1;
    grnVal = 256 - potVal;
    bluVal = potVal;
  }
  else
  {
    potVal = ((potVal - 683) * 3) / 4;
    redVal = potVal;
    grnVal = 1;
    bluVal = 256 - potVal;
  }
  analogWrite(pin_cor_r, redVal);
  analogWrite(pin_cor_g, grnVal);
  analogWrite(pin_cor_b, bluVal);

}




