#include <M5Atom.h>
#include <SPI.h>
#include "MCP3004.h"
#include <math.h>
#include <WiFi.h>
// #include "Ambient.h"
#include <WiFiUdp.h>

#define TIMER0 0

#define PERIOD 1           // 送信間隔(秒)
#define SAMPLE_PERIOD 1     // サンプリング間隔(ミリ秒)
#define SAMPLE_SIZE 100     // 1ms x 100 = 100ms

WiFiClient client;
//Ambient ambient;
uint32_t chipId=0;

const char* ssid = "SSID";         // WiFi SSID
const char* password = "pass";     // WiFi パスワード
byte host[] = {192, 168, 1, 23};   // UDP InfluxDB Server address
int port = 8089;
WiFiUDP udp;

hw_timer_t * samplingTimer = NULL;

const int MCP3004_CS = 22;
MCP3004 mcp3004(MCP3004_CS);

const float rl = 100.0; // Load Resistance

volatile int t0flag;

void IRAM_ATTR onTimer0() {
    t0flag = 1;
}

// chのチャネルをサンプリングする

float ampRead(uint8_t ch) {
    int vt;
    float amp, ampsum;
    ampsum = 0;

    timerAlarmEnable(samplingTimer);  // タイマを動かす
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        t0flag = 0;
        while (t0flag == 0) {  // タイマでt0flagが1になるのを待つ
            delay(0);
        }
        vt = mcp3004.read(ch);  // chの電圧値を測る
        amp = (float)(vt - 512) / 1024.0 * 3.3 / rl * 2000.0;  // 電流値を計算
        ampsum += amp * amp;  // 電流値を2乗して足し込む
    }
    timerAlarmDisable(samplingTimer);  // タイマを止める

    return ((float)sqrt((double)(ampsum / SAMPLE_SIZE)));  // 電流値の2乗を平均し、平方根を計算
}

// リングバッファ
#define NDATA 100  // リングバッファの件数
struct d {
    bool valid;
    float d1;
    float d2;
} data[NDATA];  // リングバッファ
int dataIndex = 0;  // リングバッファのインデクス

void putData(float d1, float d2) {  // リングバッファにデータを挿入する
    if (++dataIndex >= NDATA) {
        dataIndex = 0;
    }
    data[dataIndex].valid = true;
    data[dataIndex].d1 = d1;
    data[dataIndex].d2 = d2;
}

#define X0 10

int data2y(float d, float minY, float maxY, int HEIGHT) {  // データの値からy軸の値を計算する
    return HEIGHT - ((int)((d - minY) / (maxY - minY) * (float)HEIGHT) + 1);
}

void setup(){
    M5.begin(true,false,true);
    delay(50);
    M5.dis.fillpix(0xff0000);
    SPI.begin(23,33,19,-1);
 for(int i=0; i<17; i=i+8) {
 chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
 }
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());

    mcp3004.begin();

    samplingTimer = timerBegin(TIMER0, 80, true);
    timerAttachInterrupt(samplingTimer, &onTimer0, true);
    timerAlarmWrite(samplingTimer, SAMPLE_PERIOD * 1000, true);

    for (int i; i < NDATA; i++) {
        data[i].valid = false;
    }
}

void loop() {
    M5.dis.fillpix(0x00ff00);
    unsigned long t = millis();  // 開始時刻を記録する
    float a0, a1;
    a0 = ampRead(0);  // 系統1の電流値を測定する
    a1 = ampRead(1);  // 系統2の電流値を測定する

    putData(a0, a1);  // 電流値をリングバッファに挿入する

  while ((millis() - t) < PERIOD * 1000) {
        delay(0);
    }
    String line;
    line="ampere device="+String(chipId)+",A0="+a0+",A1="+a1;
    udp.beginPacket(host, port);
    udp.print(line);
    udp.endPacket();
    Serial.println(line);
    M5.dis.fillpix(0x0000ff);
    delay(100);
}
