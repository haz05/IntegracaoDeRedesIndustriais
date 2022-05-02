#include <Ethernet.h>
#include <SPI.h>

int data;
int sensor;
int level_value;
int start;
int stops;
int setpoint;
float error;

float term_P;
float term_I;
float term_D;
int PID_value;
int selo;
int medida[8];
float delta_temp;
long temp_pass;
long temp_print;
char msg;
char c;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 169, 254, 190, 180 };
byte server[] = { 169, 254, 190, 201 }; // Google



  
  
int kp = 50;
float ki = 1.2;

EthernetClient client;


void setup()
{
  Ethernet.begin(mac, ip);
  Serial.begin(9600);

  delay(1000);

  //Serial.println("connecting...");

  if (client.connect(server, 502)) {
 //   Serial.println("connected");
  } else {
  //  Serial.println("connection failed");
  }
}

void loop()
{    
    
    
    serialEvent();
    ReadInput(1,"Coil",2,&stops);
    ReadInput(1,"Coil",0,&start);

    
    
     if (start == 1)                                                ///< Botão start pressionado
     {
      WriteReg(1,0,0);
      selo = 1; 
     }
     if (selo == 1)                                                ///< Botão start pressionado
     {
      WriteCoil(1,0,0xFF);                                        ///< Botão start ligado
      WriteCoil(1,2,0);                                           ///< Botão stop desligado
      controlON();                                                ///< chama função de controle 
     }
     if (stops == 0)                                                ///< Botão start pressionado
     {
      WriteReg(1,0,0);                                             ///< fecha a valvula
      WriteReg(1,3,0);                                             ///< Esreve no display PV
      WriteCoil(1,2,0xFF);                                         ///< Botão stop ligado
      WriteCoil(1,0,0);                                            ///< Botão start desligado
      selo = 0; 
     }                                            



}


void controlON()
{
  ReadInput(1,"Reg",2,&setpoint);                                       ///< Aquisita valor do potenciometro de setpoint da planta
  ReadInput(1,"Reg",0,&level_value);                                    ///< Aquisita valor do nível do tanque
  WriteReg(1,2,setpoint);                                               ///< Escreve o setpoint no display
  
  error = setpoint-level_value;                                         ///< Calcula o erro atual
  delta_temp = ((millis() - temp_pass)/1000.0);
  temp_pass = millis();
  
  term_P = kp * error;                                                   ///< Calcula o termo proporcional

  term_I = term_I + (error *ki) * (delta_temp);                                  ///< Calcula o termo integrral

  PID_value = term_P + term_I;                                          ///< Soma os termos

  if (PID_value >999)
  {
   PID_value = 1000;                                                    ///< Seta limite superior
  }
  
  if (PID_value <=0)
  {
   PID_value = 0;                                                    ///< Seta limite inferior
  }
  WriteReg(1,0,PID_value);                                              ///< Esreve na válvula de abertura
  WriteReg(1,3,PID_value);                                             ///< Esreve no display PV

     
     medida[0] = setpoint;
     medida[1] = level_value;
     medida[2] = error;
     medida[3]= PID_value;
     medida[4]= kp;
     if ((millis()-temp_print)>100)
     {
        for(int i=0; i<5; i++) {
          Serial.print(medida[i]);
          Serial.print(" ");
        }
        Serial.println(ki*100);
        temp_print = millis();
      }
  }


//funcao para ler do modbus 
void ReadInput(char ID,String tip, int InputAddr,int *data)
{
  int mode;
  if (tip == "Coil")
  {
    mode = 0x02;
  }
  else if(tip == "Reg")
  {
    mode = 0x04;
  }
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,mode,InputAddr>>8,InputAddr&0x00FF,0x00,0x01};           ///< {num escravo, função (escrita ou leitura),end posição de memoria, , quantidade de operandos, ,dados a serem escritos, controle erros, ,}
  client.write(buf,12);                                                                                     ///< transaction identifier (2 bytes) - protocol identifier (2 bytes) - lenght (2 bytes)
  char resp[11];
  int i=0;
  while(!client.available());
  while(client.available())
  {
    resp[i] = client.read();
    i++;
  }

  if (tip == "Coil")
  {
    *data = resp[9];
  }
  else if(tip == "Reg")
  {
    *data = resp[9]<<8 | resp[10]&0xFF;
  }
}


//função para escrever coil (boleano 0 ou 1)

void WriteCoil(char ID, int InputAddr, int bool_value)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x05,InputAddr>>8,InputAddr&0x00FF,bool_value,0x00};

  client.write(buf,12);
  while(!client.available());
  while(client.available())
  char c = client.read();
  
}

void WriteReg(char ID,int RegAddr, int reg_value)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x06,RegAddr>>8,RegAddr&0x00FF,reg_value>>8,reg_value&0x00FF};
  client.write(buf,12);
  while(!client.available());
  while(client.available())
    char c = client.read();
}

void serialEvent() {
  String msg = "";
  while(Serial.available() > 0) {
    msg += (char)Serial.read();
    delay(2);
  }
  decide(msg);
}
void decide(String msg){
    if(msg.substring(0,2)=="P="){
      String val = msg.substring(2,4);
      kp = val.toInt();
    }
    else if(msg.substring(0,2)=="I="){
       String val = msg.substring(2,5);
      ki = val.toInt()/100.0;
    }
}
