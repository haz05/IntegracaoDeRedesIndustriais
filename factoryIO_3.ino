#include <Ethernet.h>
#include <SPI.h>

int data;
int sensor;
int level_value;
char msg;
char c;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 169, 254, 190, 180 };
byte server[] = { 169, 254, 190, 201 }; // Google


EthernetClient client;


void setup()
{
  Ethernet.begin(mac, ip);
  Serial.begin(9600);

  delay(1000);

  Serial.println("connecting...");

  if (client.connect(server, 502)) {
    Serial.println("connected");
  } else {
    Serial.println("connection failed");
  }
}

void loop()
{    
    
    
    lend();
    ReadInput(1,"Coil",3,&data);
    ReadInput(1,"Reg",2,&sensor);
    ReadInput(1,"Reg",0,&level_value);
    WriteReg(1,2,sensor);
    WriteReg(1,0,sensor);
    if (data == 1)
    {
     Serial.print("Run:");
     Serial.print(data);
     Serial.print(" Level meter:");
     Serial.print(level_value);
     Serial.print(" Sensor:");
     Serial.println(sensor);

     if(level_value > 200)
     {
      WriteReg(1,1,1000);
      }
      else if(level_value < 200)
     {
      WriteReg(1,1,0);
      }
     
    }
    else if(data ==0)
    {
    
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


void lend(){
    char msg = Serial.read();
    if(msg == 's')
    {
      ReadInput(1,"Coil",3,&data);
    }
    else if (msg == 'd')
    {
      MbReadCoil(0x01,0x03,&data);
    }
  }





void MbReadCoil(char ID,int RegAddr,int *data)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x02,RegAddr>>8,RegAddr&0x00FF,0x00,0x01};
  client.write(buf,12);
    for (int i=0; i<11; i++)
  {
    Serial.print(buf[i],HEX);
    Serial.print(" ");
    }
  Serial.println("");  
  char resp[11];
  int i=0;
  while(!client.available());
  while(client.available())
  {
    resp[i] = client.read();
    i++;
  }
  *data = resp[9];
}


void CoilWrite(char ID, int InputAddr, int bool_value)
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
