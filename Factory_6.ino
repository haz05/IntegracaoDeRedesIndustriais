#include <Ethernet.h>
#include <SPI.h>

#define AFD 0.9
#define BFD 0.1
bool conect = false;
bool conect_1 = false;
bool led_ligar = false;
bool led_desligar = false;

int valvula_saida;
int nivel; 
int controle_auto;
int controle_manual;
int data;
int sensor;
int level_value;
int start;
int stops;
int setpoint;
int mv_actual_sat;
int mv_actual;
int emergencia;
int tensao_Vsaida;
int tensao_Ventrada;

float error;

float term_P;
float term_I;
float term_D;
int PID_value;
int selo;
int medida[9];
float delta_temp;
long temp_pass;
long temp_print;
char msg;
char c;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 169, 254, 190, 200 };
byte server[] = { 169, 254, 190, 201}; // escravo

int abertura_enchimento;
int abertura_saida;
float error_level_ant;
float error_level;
float xd_level;
float pv_actual;
float kp = 6;  
float ki = 0.04;  
float pv_actual_sat;
float xi_level;
float kw = 0.01;

EthernetClient client;


void setup()
{
  Ethernet.begin(mac, ip);
  Serial.begin(9600);

  delay(1000);

  //Serial.println("connecting...");

  if (client.connect(server, 502)) {
    //Serial.println("connected");
    conect_1 = true;
  } else {
    //Serial.println("connection failed");
  }

  if (conect_1)
  {
    WriteCoil(1,0,0);                                          ///< LED Botão Ligar desligado
    WriteCoil(1,1,0);                                          ///< LED Botão desligar desligado
    WriteCoil(1,2,0);                                          ///< LED Planta ativa desligado
    WriteCoil(1,3,0xFF);                                       ///< LED Planta desativada ligado
  }
}

void loop()
{    
  ReadInput(1,"Input",4,&emergencia);
  ReadInput(1,"Reg",3,&nivel);                                            ///< Lê valor do sensor de nível
  WriteReg(1,2,nivel);                                                    ///< Escreve no display de nível
  
  if(emergencia)
  {
    //serialEvent();
    ReadInput(1,"Input",1,&stops);
    ReadInput(1,"Input",0,&start);

    
    


     if (start == 1)                                                ///< Botão start pressionado
     {
        WriteCoil(1,0,0xFF);                                        ///< LED Botão Ligar ligado
        led_ligar = true;
        ReadInput(1,"Input",2,&controle_auto);
        ReadInput(1,"Input",3,&controle_manual);
      //WriteReg(1,0,0);
        selo = 1;
        WriteCoil(1,0,0xFF); 
     }
     else if(led_ligar && start == 0)
     {
        led_ligar = false;
        WriteCoil(1,0,0);                                          ///< LED Botão Ligar desligado
     }
     if(stops == 1)
     {
        led_desligar = true;
        WriteCoil(1,1,0xFF);                                          ///< LED Botão desigar ligado
        selo = 0;
     }
     else if (led_desligar && stops == 0)
     {
        led_desligar = false;
        WriteCoil(1,1,0);                                          ///< LED Botão Ligar desligado
      }

      if(selo)
      {
          WriteCoil(1,2,0xFF);                                     ///< LED Planta ativa ligado
          WriteCoil(1,3,0);                                       ///< LED Planta desativada desligado
          if (controle_auto)
          {
            WriteCoil(1,4,0xFF);                                   ///< LED controle automatico ligado
            levelControl();                                                ///< chama função de controle  
          }
          if (controle_manual)
          {
            WriteCoil(1,5,0xFF);                                  ///< LED controle manual ligado
            manualControl();                                       ///< chama função de controle manual 
          }
      }
      else
      {
        WriteReg(1,0,0);                                         ///< Escreve no display de entrada
        WriteReg(1,1,0);                                         ///< Escreve no display de saida
        WriteCoil(1,2,0);                                          ///< LED Planta ativa desligado
        WriteCoil(1,3,0xFF);                                       ///< LED Planta desativada ligado
        WriteCoil(1,4,0);                                          ///< LED controle automatico desligado
        WriteCoil(1,5,0);                                           ///< LED controle manual desligado
        WriteReg(1,5,0);                                            ///< Esreve na valvula de entrada
        WriteReg(1,6,0);                                             ///< Esreve na valvula de saida
      }
      
  }
  else
  {
    if (nivel > 0)
    {
      WriteReg(1,6,1000);                                             ///< Esreve na valvula de saida
    }
    else if(nivel == 0)
    {
      WriteReg(1,6,0);                                             ///< Esreve na valvula de saida
    }
        WriteReg(1,0,0);                                         ///< Escreve no display de entrada
        WriteReg(1,1,0);                                         ///< Escreve no display de saida
        WriteCoil(1,2,0);                                          ///< LED Planta ativa desligado
        WriteCoil(1,3,0xFF);                                       ///< LED Planta desativada ligado
        WriteCoil(1,4,0);                                          ///< LED controle automatico desligado
        WriteCoil(1,5,0);                                           ///< LED controle manual desligado
        WriteReg(1,5,0);                                            ///< Esreve na valvula de entrada
  }

      
    /*
     if(stops == 10)
     {
      Serial.println(stops);
      WriteCoil(1,1,0xFF);                                          ///< Botão start ligado
     }
     if (selo == 1)                                                ///< Botão start pressionado
     {
      WriteCoil(1,0,0xFF);                                        ///< Botão start ligado
      WriteCoil(1,2,0);                                           ///< Botão stop desligado
      //levelControl();                                                ///< chama função de controle 
     }
     if (stops == 0)                                                ///< Botão start pressionado
     {
      //WriteReg(1,0,0);                                             ///< fecha a valvula
      //WriteReg(1,3,0);                                             ///< Esreve no display PV
      //WriteCoil(1,2,0xFF);                                         ///< Botão stop ligado
      //WriteCoil(1,0,0);                                            ///< Botão start desligado
      selo = 0; 
     }                                            

*/
env_serial();
}

void env_serial()
{
  if (selo && emergencia)
  {
    
     medida[0] = controle_auto;
     medida[1] = controle_manual;
     medida[2] = selo;
     medida[3] = tensao_Ventrada;
     medida[4]= tensao_Vsaida;
     medida[5]= nivel;
     medida[6]= setpoint;
     medida[7]= emergencia;
     medida[8]= kp;
  }
   else
   {
     medida[0] = 0;
     medida[1] = 0;
     medida[2] = 0;
     medida[3] = 0;
     medida[4]= 0;
     medida[5]= nivel;
     medida[6]= 0;
     medida[7]= emergencia;
     medida[8]= kp;
    }
     if ((millis()-temp_print)>100)
     {
        for(int i=0; i<9; i++) {
          Serial.print(medida[i]);
          Serial.print(" ");
        }
        Serial.println(ki*100);
        temp_print = millis();
      }
      

  }
void levelControl()
{  
  
  ReadInput(1,"Reg",2,&setpoint);                                       ///< Aquisita valor do potenciometro de setpoint da planta
  ReadInput(1,"Reg",5,&valvula_saida);                                  ///< Aquisita valor do potenciometro da valvula de saida da planta
  
  WriteReg(1,3,setpoint);                                               ///< Escreve o setpoint no display
  WriteReg(1,6,valvula_saida);                                             ///< Esreve na valvula de saida

  ReadInput(1,"holding",6,&tensao_Vsaida);                              ///< Lê a tensao aplicada a valvula
  
  WriteReg(1,7,tensao_Vsaida);                                          ///< Escreve no display da valvula de saida

   error_level_ant = error_level;
   error_level = setpoint - nivel;
    
   mv_actual = kp * error_level + xi_level;
   mv_actual_sat = mv_actual;

    if (mv_actual_sat < 0)
    {
      mv_actual_sat = 0;
    }
    else if (mv_actual_sat > 999)
    {
      mv_actual_sat = 1000;
    }
  
  WriteReg(1,5,mv_actual_sat);                                             ///< Esreve na valvula de entrada

  ReadInput(1,"holding",5,&tensao_Ventrada);                              ///< Lê a tensao aplicada a valvula
  WriteReg(1,4,tensao_Ventrada);                                             ///< Esreve na tensao no display 
  
  xi_level = xi_level + ki * error_level - kw * (mv_actual - mv_actual_sat);  
      
}

void manualControl()
{  
  ReadInput(1,"Reg",0,&abertura_enchimento);                              ///< Lê valor do potencioemtrro de entrada
  ReadInput(1,"Reg",1,&abertura_saida);                                    ///< Lê valor do potencioemtrro de saida
  
  WriteReg(1,5,abertura_enchimento);                                        ///< Esreve na valvula de entrada
  WriteReg(1,6,abertura_saida);                                             ///< Esreve na valvula de saida


  ReadInput(1,"holding",5,&tensao_Ventrada);                              ///< Lê a tensao aplicada a valvula  
  ReadInput(1,"holding",6,&tensao_Vsaida);                             ///< Lê a tensao aplicada a valvula       
  WriteReg(1,0,tensao_Ventrada);                                      ///< Escreve no display de entrada
  WriteReg(1,1,tensao_Vsaida);                                           ///< Escreve no display de saida
  
  //Serial.println(abertura_enchimento);
  //WriteReg(1,0,pv_actual_sat);                                              ///< Esreve na válvula de abertura
  //WriteReg(1,3,pv_actual_sat);                                             ///< Esreve no display PV
  

  
  //Serial.print(level_value);
  //Serial.print("||");
  //Serial.println(setpoint);
     medida[0] = setpoint;
     medida[1] = level_value;
     medida[2] = error;
     medida[3]= PID_value;
     medida[4]= kp;
     /*
     if ((millis()-temp_print)>100)
     {
        for(int i=0; i<5; i++) {
          Serial.print(medida[i]);
          Serial.print(" ");
        }
        Serial.println(ki*100);
        temp_print = millis();
      }
      */
}


void controlON()
{
  ReadInput(1,"Reg",2,&setpoint);                                       ///< Aquisita valor do potenciometro de setpoint da planta
  ReadInput(1,"Reg",0,&level_value);                                    ///< Aquisita valor do nível do tanque
  WriteReg(1,2,setpoint);                                               ///< Escreve o setpoint no display

  Serial.println(setpoint);
  error = setpoint-level_value;                                         ///< Calcula o erro atual
  delta_temp = ((millis() - temp_pass)/1000.0);
  temp_pass = millis();
  
  term_P = kp * error;                                                   ///< Calcula o termo proporcional

  term_I = term_I + (error *ki) * (delta_temp);                           ///< Calcula o termo integrral

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
          //Serial.print(medida[i]);
          //Serial.print(" ");
        }
        //Serial.println(ki*100);
        temp_print = millis();
      }
  }


//funcao para ler do modbus 
void ReadInput(char ID,String tip, int InputAddr,int *data)
{
  int mode;
  if (tip == "Input")         ///< discreto
  {
    mode = 0x02;
  }
  else if (tip == "holding")
  {
    mode = 0x03;
  }
  else if(tip == "Reg")     ///< valor em 2 bytes
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

  if (tip == "Input")
  {
    *data = resp[9];
  }
  else if(tip == "Reg" || tip == "holding")
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
