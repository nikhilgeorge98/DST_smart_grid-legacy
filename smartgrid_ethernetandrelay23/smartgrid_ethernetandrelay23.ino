 

#include <SimpleModbusMasterSDM120.h>
#include <UIPEthernet.h>

EthernetUDP udp;
unsigned long next;


// Direcciones registros de datos solo lectura. Valores tipo float.
// Utilizar funcion 04 lectura, numero de bytes 4.
// Las marcadas en MAYUSCULAS son las de el este ejemplo.

 #define VOL_ADR 0X0000    // VOLTAJE.
 #define CUR_ADR 0X0006    // CORRIENTE.
 #define POW_ADR 0X000C    // POTENCIA ACTIVA. 
 #define VAM_ADR 0X0012    // Potencia Aparente.
 #define PRE_ADR 0X0018    // Potencia Reactiva.
 #define PFA_ADR 0X001E    // Factor de potencia.
 #define FRE_ADR 0X0046    // Frecuencia.
 #define ENE_ADR 0X0048    // ENERGIA IMPORTADA KWH
 #define POE_ADR 0X004A    // Energia exportada.
 #define NEE_ADR 0X0156    // Energia activa Total.
//

/*
Constants are provided for:
  Function 1  - READ_COIL_STATUS
  Function 2  - READ_INPUT_STATUS
  Function 3  - READ_HOLDING_REGISTERS 
  Function 4  - READ_INPUT_REGISTERS
  Function 15 - FORCE_MULTIPLE_COILS
  Function 16 - PRESET_MULTIPLE_REGISTERS 

   Valid modbus byte formats are:
    SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
    SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
    SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
*/
// Estos parametros son un poco confusos dependiendo del manual la opción por defecto puede cambiar.
// Verificar que realmente tiene estos parametros.
 #define SDM120C_METER_NUMBER   1                
 #define SDM120C_BAUDRATE       2400
 #define SDM120C_BYTEFORMAT     SERIAL_8N2    //Prty n
// 


 #define TIMEOUT 1000
 #define POLLING 2000    // the scan rate 
 #define RETRYCOUNT 10   // numero de reintentos, para volver set the "connection" variable to true.
 #define TXENPIN  17     // Pin cambio recibir/trasmiste para el driver RS485


// Direcciones registros de configuración lectura/escritura  Valores tipo HEX.
// Utilizar funcion 10 escritura, 04 lectura numeor de bytes 1.

// IMPLEMENTAR.

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
 PACKET1,
 PACKET2,
 PACKET3,
 PACKET4,
 PACKET5,
 TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Create a packetPointer to access each packet
// individually. This is not required you can access
// the array explicitly. E.g. packets[PACKET1].id = 2;
// This does become tedious though...
packetPointer volPacket = &packets[PACKET1];
packetPointer curPacket = &packets[PACKET2];
packetPointer powPacket = &packets[PACKET3];
packetPointer enePacket = &packets[PACKET4]; 
packetPointer pfPacket =  &packets[PACKET5];
// Union 
union datas{
 byte  b[4];
 float f;
 unsigned int Array[2]; 
}voltage, current ,power, energy,powerfactor;
int datafromUser=0;
char V[255],I[255],P[255],E[255],PF[255],ST[255];
char incomingPacket[255];
int data=0;

void setup() {

 // modbus_construct(packet, id, function, address, data, register array)    
 // For functions 1 & 2 data is the number of points
 // For functions 3, 4 & 16 data is the number of registers
 // For function 15 data is the number of coils
 modbus_construct_SDM120(volPacket, SDM120C_METER_NUMBER, VOL_ADR, voltage.Array);
 modbus_construct_SDM120(curPacket, SDM120C_METER_NUMBER, CUR_ADR, current.Array);
 modbus_construct_SDM120(powPacket, SDM120C_METER_NUMBER, POW_ADR, power.Array);
 modbus_construct_SDM120(enePacket, SDM120C_METER_NUMBER, ENE_ADR, energy.Array);
 modbus_construct_SDM120(pfPacket, SDM120C_METER_NUMBER, PFA_ADR, powerfactor.Array);
 //aPower = aPwr.read(POW_FAC);
 //PFACTOR
  //pwrFact
 /* Initialize communication settings:
    parameters(HardwareSerial* SerialPort,
   long baud, 
   unsigned char byteFormat,
   unsigned int timeout, 
   unsigned int polling, 
   unsigned char retry_count, 
   unsigned char TxEnablePin,
   Packet* packets, 
   unsigned int total_no_of_packets);

    Valid modbus byte formats are:
    SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
    SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
    SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
    
    You can obviously use SERIAL_8N1 but this does not adhere to the
    Modbus specifications. That said, I have tested the SERIAL_8N1 option 
    on various commercial masters and slaves that were suppose to adhere
    to this specification and was always able to communicate... Go figure.
    
    These are already defined in the Arduino global name space. 
 */ 
 // Iniciamos comunicación modbus SERIAL1 Arduino Mega.
 modbus_configure(&Serial1, SDM120C_BAUDRATE, SDM120C_BYTEFORMAT, TIMEOUT, POLLING, RETRYCOUNT, TXENPIN, packets, TOTAL_NO_OF_PACKETS);
 
 //Iniciamos puerto serial"0" Arduino Mega para visualizar datos.
 Serial.begin(9600);
 Serial.println(F("Test SDM120-Modbus"));
 pinMode(4, OUTPUT);  // Led pin de Arduino Mega;
 digitalWrite(4, HIGH);
 pinMode(5,OUTPUT);
 digitalWrite(5, HIGH);
 uint8_t mac[6] = {0x00,0x01,0x03,0x04,0x02,0x05};
//Ethernet.begin(mac,IPAddress(192,168,1,37));
  Ethernet.begin(mac,IPAddress(172,1,14,27));
  int success = udp.begin(7777);

}

void loop() {
 static boolean b = false ;
 static unsigned long oldMillis;  
 modbus_update();
 
  
// if( (millis()-oldMillis) > POLLING){
   unsigned long temp;
   float v,i,p,e,pf;
   digitalWrite(13,b);
   b++;    
   oldMillis=millis();
   
   //Serial.println("    MEDIDAS:");
   
   v = (float)voltage.f;
   //Serial.print(F(" Voltaje="));
   Serial.print(v, 1);
   Serial.print(",");
   
   i= (float)current.f*10;
   //Serial.print(F(" Corriente="));
   Serial.print(i, 2);
   Serial.print(",");
   
   p= (float)power.f*10;
   //Serial.print(F(" Potencia="));
   Serial.print(p, 1);
   Serial.print(",");
   
   e= (float)energy.f;
   //Serial.print(F(" Energia="));
   Serial.print(e, 2);  
   
    Serial.print(",");
   
   pf= (float)powerfactor.f;
   //Serial.print(F(" Energia="));
   
   Serial.print(pf);
   Serial.print(",");
   Serial.println(data);
   dtostrf(v,3,2,V);
   dtostrf(i,3,2,I);
   dtostrf(p,3,2,P);
   dtostrf(e,3,2,E);
   dtostrf(pf,3,2,PF);
   dtostrf(data,3,2,ST);

    
 if(Serial.available() > 0)
  {
    datafromUser=Serial.read();
  }

  if(data == 0)
  {
    digitalWrite(4 , HIGH );
    digitalWrite(5 , HIGH );
//    digitalWrite(LED_BUILTIN, HIGH);
  }
 else if(data == 1)
  {
    digitalWrite( 4, LOW);
    digitalWrite( 5, LOW);
  }
  
  udp.beginPacket(IPAddress(172,1,28,85),7777);  //172,1,28,57
     udp.write(V);
    udp.write(",");
    udp.write(I);
     udp.write(",");
      udp.write(P);
    udp.write(",");
      udp.write(E);
    udp.write(",");
     udp.write(PF);
     udp.write(",");
     udp.write(ST);
     udp.endPacket();

     udp.beginPacket(IPAddress(172,1,26,131),7777);
 //udp.beginPacket(IPAddress(192,168,1,216),7000);//172,1,28,57
     udp.write(V);
    udp.write(",");
    udp.write(I);
    udp.write(",");
      udp.write(P);
    udp.write(",");
      udp.write(E);
    udp.write(",");
     udp.write(PF);
     udp.write(",");
     udp.write(ST);
     udp.endPacket();

     udp.beginPacket(IPAddress(172,1,26,108),7777);
 //udp.beginPacket(IPAddress(192,168,1,216),7000);//172,1,28,57
     udp.write(V);
    udp.write(",");
    udp.write(I);
    udp.write(",");
      udp.write(P);
    udp.write(",");
      udp.write(E);
    udp.write(",");
     udp.write(PF);
     udp.write(",");
     udp.write(ST);
     udp.endPacket();

          udp.beginPacket(IPAddress(172,1,14,104),7777);
 //udp.beginPacket(IPAddress(192,168,1,216),7000);//172,1,28,57
     udp.write(V);
    udp.write(",");
    udp.write(I);
    udp.write(",");
      udp.write(P);
    udp.write(",");
      udp.write(E);
    udp.write(",");
     udp.write(PF);
     udp.write(",");
     udp.write(ST);
     udp.endPacket();
//    delay(300);

  int size = udp.parsePacket();
  if (size > 0) {
    do
      {
        char* msg = (char*)malloc(size+1);
        int len = udp.read(msg,size+1);
        msg[len]=0;
        Serial.print("received: '");
        Serial.print(msg);
        data=atoi(msg);
        free(msg);
        
      }
    while ((size = udp.available())>0);
    
  
  }
}
// } 
//}
