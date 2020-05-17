#include "mbed.h"
#include "mbed_rpc.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "math.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#define PI acos(-1)

#define UINT14_MAX 16383
// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E << 1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D << 1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C << 1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F << 1) // with pins SA0=1, SA1=1
// FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7

WiFiInterface *wifi = WiFiInterface::get_default_instance();
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
const char* topic = "Mbed";
int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
NetworkInterface* net = wifi;
MQTTNetwork mqttNetwork(net);
MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

I2C i2c(PTD9, PTD8);
Serial pc(USBTX, USBRX);
Serial xbee(D12, D11);
EventQueue detectOver45_queue(32*EVENTS_EVENT_SIZE);
EventQueue collectData_queue(32*EVENTS_EVENT_SIZE);
EventQueue mqtt_queue(32*EVENTS_EVENT_SIZE);
EventQueue RPC_queue(32*EVENTS_EVENT_SIZE);
Thread detectOver45_thread(osPriorityNormal);
Thread collectData_thread(osPriorityNormal);
Thread mqtt_thread(osPriorityHigh);
Thread RPC_thread(osPriorityNormal);
InterruptIn btn3(SW3);
Timer t;
Timer count20Sec;



int m_addr = FXOS8700CQ_SLAVE_ADDR1;
void FXOS8700CQ_readRegs(int addr, uint8_t *data, int len);
void FXOS8700CQ_writeRegs(uint8_t *data, int len);
void getAcc(float *x, float *y , float *z);
void accelerometer();
void getCollectDataTime(Arguments *in, Reply *out);
bool tilt_over_45(float x,float y,float z);
void collectData();
void publish_message(float x);
void messageArrived(MQTT::MessageData& md);
void close_mqtt();
void RPC_function();

RPCFunction rpcGetCollect(&getCollectDataTime,"getCollectDataTime");

bool now_over_45 = false;
bool last_over_45 = false;
bool BecomeBiggerThan45 = false;
int collectCount = 0;
int dataNum = 0;
bool XBeeHostStart = true;

typedef struct POS{
    float x;
    float y;
    float z;
    float time;
}POS;

POS point[300];

void publish_message(float x) {
      //message_num++;
      MQTT::Message message;
      char buff[100];
      sprintf(buff,"%f",x);
      message.qos = MQTT::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) buff;
      message.payloadlen = strlen(buff) + 1;
      int rc = client.publish(topic, message);
      printf("rc:  %d\r\n", rc);
      printf("Puslish message: %s\r\n", buff);
}
void messageArrived(MQTT::MessageData& md) {
      MQTT::Message &message = md.message;
      char msg[300];
      sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
      printf(msg);
      wait_ms(1000);
      char payload[300];
      sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
      printf(payload);
      ++arrivedcount;
}
bool tilt_over_45(float x,float y,float z){
    float angle = atan(z/sqrt(x*x+y*y))*180/PI;
    if(angle<=45){
        return true;
    }
    else {
        return false;
    }      
}

void reply_messange(char *xbee_reply, char *messange)
{
    xbee_reply[0] = xbee.getc();
    xbee_reply[1] = xbee.getc();
    xbee_reply[2] = xbee.getc();
    if (xbee_reply[1] == 'O' && xbee_reply[2] == 'K')
    {
        pc.printf("%s\r\n", messange);
        xbee_reply[0] = '\0';
        xbee_reply[1] = '\0';
        xbee_reply[2] = '\0';
    }
}
void check_addr(char *xbee_reply, char *messenger)
{
    xbee_reply[0] = xbee.getc();
    xbee_reply[1] = xbee.getc();
    xbee_reply[2] = xbee.getc();
    xbee_reply[3] = xbee.getc();
    pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
    xbee_reply[3] = '\0';
}

void getAcc(float *x, float *y , float *z)
{
    int16_t acc16;
    float t[3];
    uint8_t res[6];
    FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);
    acc16 = (res[0] << 6) | (res[1] >> 2);
    if (acc16 > UINT14_MAX / 2)
        acc16 -= UINT14_MAX;
    t[0] = ((float)acc16) / 4096.0f;
    acc16 = (res[2] << 6) | (res[3] >> 2);
    if (acc16 > UINT14_MAX / 2)
        acc16 -= UINT14_MAX;
    t[1] = ((float)acc16) / 4096.0f;
    acc16 = (res[4] << 6) | (res[5] >> 2);
    if (acc16 > UINT14_MAX / 2)
        acc16 -= UINT14_MAX;
    t[2] = ((float)acc16) / 4096.0f;
    *x = t[0];
    *y = t[1];
    *z = t[2]; 
}

void collectData(){
    t.reset();
    t.start();
    wait(1);
    count20Sec.reset();
    count20Sec.start();
    while(count20Sec.read()<=22){
        getAcc(&point[dataNum].x,&point[dataNum].y,&point[dataNum].z);
        point[dataNum].time = count20Sec.read_ms();
        dataNum++;
        collectCount++;
        if(BecomeBiggerThan45||t.read()<1){
            if(BecomeBiggerThan45){
                t.reset();
                t.start();
            }      
            wait(0.1);
        }
        else{
            wait(0.5);
        }
    }
    pc.printf("dataNum = %d ",dataNum);
    wait(2);
    XBeeHostStart = true;
    collectCount = 0;
    //mqtt_queue.call(publish_message,dataNum);
    //wait(0.1);
    for(int j=0;j<dataNum;j++){
        mqtt_queue.call(publish_message,point[j].x);
        wait(0.1);
        mqtt_queue.call(publish_message,point[j].y);
        wait(0.1);
        mqtt_queue.call(publish_message,point[j].z);
        wait(0.1);
        mqtt_queue.call(publish_message,point[j].time);
        wait(0.1);
    } 
    mqtt_queue.call(publish_message,99999);
        wait(0.1);
    dataNum = 0;
}

void getCollectDataTime(Arguments *in, Reply *out){
    if(XBeeHostStart){
        collectData_queue.call(collectData);
        XBeeHostStart = false;
    }       
    char collect_str[3];
    sprintf(collect_str,"%02d",collectCount);
    //xbee.printf("%s",collect_str);
    // char test[3];
    // test[0] = 'a';
    // test[1] = 'b';
    // test[2] = '\0';
    xbee.printf("%s",collect_str);
    collectCount = 0;
}

void detectOver45(){
    int16_t acc16;
    float t[3];
    uint8_t res[6];
    while(1){
        FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);
        acc16 = (res[0] << 6) | (res[1] >> 2);

        if (acc16 > UINT14_MAX/2)
            acc16 -= UINT14_MAX;
        t[0] = ((float)acc16) / 4096.0f;

        acc16 = (res[2] << 6) | (res[3] >> 2);
        if (acc16 > UINT14_MAX/2)
            acc16 -= UINT14_MAX;
        t[1] = ((float)acc16) / 4096.0f;

        acc16 = (res[4] << 6) | (res[5] >> 2);
        if (acc16 > UINT14_MAX/2)
            acc16 -= UINT14_MAX;
        t[2] = ((float)acc16) / 4096.0f;
        last_over_45 = now_over_45;
        now_over_45 = tilt_over_45(t[0],t[1],t[2]);
        BecomeBiggerThan45 = (now_over_45 ==true && last_over_45==false)? true:false;
        wait(0.5);
   }
}

void FXOS8700CQ_readRegs(int addr, uint8_t *data, int len)
{
    char t = addr;
    i2c.write(m_addr, &t, 1, true);
    i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t *data, int len)
{
    i2c.write(m_addr, (char *)data, len);
}
void close_mqtt() {
      closed = true;
}
void RPC_function(){
    char buf[256], outbuf[256];
    while (true)
    {
        memset(buf, 0, 256); // clear buffer
        for (int i = 0; i < 255; i++)
        {
            char recv = xbee.getc();
            if (recv == '\r' || recv == '\n')
            {
                pc.printf("\r\n");
                break;
            }
            buf[i] = pc.putc(recv);
            //buf[i] = recv;
        }
        RPC::call(buf, outbuf);
        pc.printf("%s\r\n", outbuf);
    }
}


int main()
{
    uint8_t data_acc[2];
    // Enable the FXOS8700Q
    FXOS8700CQ_readRegs(FXOS8700Q_CTRL_REG1, &data_acc[1], 1);
    data_acc[1] |= 0x01;
    data_acc[0] = FXOS8700Q_CTRL_REG1;
    FXOS8700CQ_writeRegs(data_acc, 2);
    
    pc.baud(9600);
    char xbee_reply[4];
    // XBee setting
    xbee.baud(9600);
    xbee.printf("+++");
    xbee_reply[0] = xbee.getc();
    xbee_reply[1] = xbee.getc();
    if (xbee_reply[0] == 'O' && xbee_reply[1] == 'K')
    {
        pc.printf("enter AT mode.\r\n");
        xbee_reply[0] = '\0';
        xbee_reply[1] = '\0';
    }
    xbee.printf("ATMY 0x265\r\n");
    reply_messange(xbee_reply, "setting MY : 0x265");
    xbee.printf("ATDL 0x165\r\n");
    reply_messange(xbee_reply, "setting DL : 0x165");
    xbee.printf("ATWR\r\n");
    reply_messange(xbee_reply, "write config");
    xbee.printf("ATMY\r\n");
    check_addr(xbee_reply, "MY");
    xbee.printf("ATDL\r\n");
    check_addr(xbee_reply, "DL");
    xbee.printf("ATCN\r\n");
    reply_messange(xbee_reply, "exit AT mode");
    //xbee.getc();
    // start
    pc.printf("start\r\n");
    //xbee.getc(); //remove the first redundant char

    //=================================wifi=================================
    
    if (!wifi) {
          printf("ERROR: No WiFiInterface found.\r\n");
          return -1;
    }
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    
    if (ret != 0) {
          printf("\nConnection error: %d\r\n", ret);
          return -1;
    }
    
    //TODO: revise host to your ip
    const char* host = "192.168.1.136";
    printf("Connecting to TCP network...\r\n");
    int rc = mqttNetwork.connect(host, 1883);
    if (rc != 0) {
          printf("Connection error.");
          return -1;
    }
    printf("Successfully connected!\r\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";
    if ((rc = client.connect(data)) != 0){
          printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
          printf("Fail to subscribe\r\n");
    }
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    //=================================wifi=================================
    detectOver45_thread.start(callback(&detectOver45_queue, &EventQueue::dispatch_forever));
    detectOver45_queue.call(detectOver45);
    collectData_thread.start(callback(&collectData_queue, &EventQueue::dispatch_forever));
    RPC_thread.start(callback(&RPC_queue,&EventQueue::dispatch_forever));
    RPC_queue.call(RPC_function);

    btn3.rise(&close_mqtt);
    int num = 0;
    while (num != 5) {
          client.yield(100);
          ++num;
    }
    while (1) {
          if (closed) break;
          wait(0.5);
    }
    printf("Ready to close MQTT Network......\n");
    if ((rc = client.unsubscribe(topic)) != 0) {
          printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
          printf("Failed: rc from disconnect was %d\n", rc);
    }
    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
    return 0;    
}
