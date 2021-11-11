#include <Ethernet.h>
#include <SPI.h>

int data;
char msg;
char c;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 169, 254, 215, 5 };
byte server[] = { 169, 254, 215, 6 }; // Google


EthernetClient client;
typedef enum
{
  Coil = 0x04,
  Register = 0x02,
  Types_request_end
}Types_request;



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
    ReadInput(0x01,"Coil",0x00,data);
    
    
    
    //MbReadInput(1,2,&data);
    MbReadCoil(1,2,&data);
    //Serial.println(data);
    if(data == 1)
    {
      //Serial.println("caralho");
    }
    
    

  if (client.available()) {
    char c = client.read();
  }
}

void MbWriteCoil_on(char ID,int RegAddr)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x05,RegAddr>>8,RegAddr&0x00FF,0xFF,0x00};
  client.write(buf,12);
  while(!client.available());
  while(client.available())
    char c = client.read();
}

void MbWriteCoil_off(char ID,int RegAddr)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x05,RegAddr>>8,RegAddr&0x00FF,0x00,0x00};
  client.write(buf,12);
  while(!client.available());
  while(client.available())
  char c = client.read();
  Serial.println(c);

   

  
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
  *data = resp[9]<<8 | resp[10]&0xFF;
  
}
/*
void MbReadInput(char ID,int RegAddr,int *data)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x04,RegAddr>>8,RegAddr&0x00FF,0x00,0x01};
  client.write(buf,12);

  char resp[11];
  int i=0;
  while(!client.available());
  while(client.available())
  {
    resp[i] = client.read();
    i++;
  }
  *data = resp[9]<<8 | resp[10]&0xFF;

}
*/
void lend(){
    char msg = Serial.read();
    if(msg == 's')
    {
      MbWriteCoil_on(1,0);
    }
    else if (msg == 'd')
    {
      MbWriteCoil_off(1,0);
    }
  }

  void MbReadCoil(char ID,int RegAddr,int *data)
{
  char buf[12] = {0x00,0x02,0x00,0x00,0x00,0x06,ID,0x02,RegAddr>>8,RegAddr&0x00FF,0x00,0x01};
  client.write(buf,12);

  char resp[11];
  int i=0;
  while(!client.available());
  while(client.available())
  {
    resp[i] = client.read();
    i++;
  }
  *data = resp[9];
  //for(int i =0; i<8;i++)
  //{
  // Serial.print(data[i]);
  //  Serial.print(" ");
 // }
  //Serial.println(data[8]);
}
