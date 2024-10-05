#include <Arduino.h>

#include <ui.h>
#include <ArduinoOTA.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <PubSubClient.h>
#include "debouncebutton.h"
#include <Arduino_GFX_Library.h>

#define GFX_BL 38
//#define GFX_BL 2

Arduino_DataBus *bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED /* DC */, 39 /* CS */,
    48 /* SCK */, 47 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    18 /* DE */, 17 /* VSYNC */, 16 /* HSYNC */, 21 /* PCLK */,
    11 /* R0 */, 12 /* R1 */, 13 /* R2 */, 14 /* R3 */, 0 /* R4 */,
    8 /* G0 */, 20 /* G1 */, 3 /* G2 */, 46 /* G3 */, 9 /* G4 */, 10 /* G5 */,
    4 /* B0 */, 5 /* B1 */, 6 /* B2 */, 7 /* B3 */, 15 /* B4 */,
    1 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    1 /* vsync_polarity */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);
  
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */, bus, GFX_NOT_DEFINED, st7701_type9_init_operations,sizeof(st7701_type9_init_operations));

/*******************************************************************************
   Please config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"

#include <esp_heap_caps.h>

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

bool lightState1=false;
bool lightState2=false;

byte powerWohnen=50;
byte powerEssen=50;
byte oldp1=powerWohnen;
byte oldp2=powerEssen;
unsigned long pUpdate=0;
unsigned long pUpdate2=0;
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  // gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      data->point.x = touch_last_y;
      data->point.y = touch_last_x;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}
/*
Ticker ticker;

void tcr1s()
{
  Serial.printf("SRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.printf("PSRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}*/

int32_t batteryLevel=0;
float price=0;
float production=0;
float power=0;
float einspeisung=0;

void updateDisplay();

void OnButtonClicked(lv_event_t *e)
{
    static uint8_t cnt = 0;
    cnt++;
    //lv_label_set_text_fmt(ui_lblCountValue, "%d", cnt);
}

void DebugPrintf(const char *, ...); //Our printf function
char *convert(int, int); //Convert integer number into octal, hex, etc.

const char *mqtt_server = "192.168.178.34";
#include "../../../wifiPasswd.h"

//WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

int networkTimeout = 0;
bool PingArrived = false;

unsigned long now=0;

//const int pButton1 = 35;
//debounceButton button1(pButton1);
#define DEBUG

#ifdef DEBUG
char linebuf[200];
unsigned int linePos = 0;
unsigned int printLine = 0;
void outputChar(char c)
{

  if (linePos < 199)
  {
    linebuf[linePos] = c;
    linePos++;
    linebuf[linePos] = '\0';
  }
  if (c == '\n')
  {
    linePos = 0;
    if (client.connected())
    {
      char top[50];
      sprintf(top, "Debug/PVDisplay2/%d", printLine);
      client.publish(top, linebuf);
      printLine++;
      if (printLine > 20)
        printLine = 0;
    }
    else
    {
      Serial.print(linebuf);
    }
    linebuf[0] = '\0';
  }
}
void outputCharp(const char *s)
{
  const char *c;
  for (c = s; *c != '\0'; c++)
  {
    outputChar(*c);
  }
}
void DebugPrintf(const char *format, ...)
{
  const char *traverse;
  int i;
  const char *s;
  char iBuf[20];

  va_list arg;
  va_start(arg, format);

  for (traverse = format; *traverse != '\0'; traverse++)
  {
    while (*traverse != '%' && *traverse != '\0')
    {
      outputChar(*traverse);
      traverse++;
    }
    if (*traverse == '\0')
    {
      break;
    }

    traverse++;

    //Module 2: Fetching and executing arguments
    switch (*traverse)
    {
    case 'c':
      i = va_arg(arg, int); //Fetch char argument
      outputChar(i);
      break;

    case 'd':
      i = va_arg(arg, int); //Fetch Decimal/Integer argument
      if (i < 0)
      {
        i = -i;
        outputChar('-');
      }
      outputCharp(itoa(i, iBuf, 10));
      break;

    case 'o':
      i = va_arg(arg, unsigned int); //Fetch Octal representation
      outputCharp(itoa(i, iBuf, 8));
      break;

    case 's':
      s = va_arg(arg, char *); //Fetch string
      outputCharp(s);
      break;

    case 'x':
      i = va_arg(arg, unsigned int); //Fetch Hexadecimal representation
      outputCharp(itoa(i, iBuf, 16));
      break;
    }
  }

  //Module 3: Closing argument list to necessary clean-up
  va_end(arg);
}

char *convert(int num, int base)
{
  static char Representation[] = "0123456789ABCDEF";
  static char buffer[50];
  char *ptr;

  ptr = &buffer[49];
  *ptr = '\0';

  do
  {
    *--ptr = Representation[num % base];
    num /= base;
  } while (num != 0);

  return (ptr);
}
#endif

void localLoop();

void callback(char *topicP, byte *payloadP, unsigned int length)
{
  char topic[200];
  char payload[200];
  strncpy(topic, topicP, 200);
  strncpy(payload, (char *)payloadP, length);
  payload[length] = '\0';

  //DebugPrintf("Message arrived [%s] %s %d %d\n", topic, payload,pUpdate2, now);
  if ((strcmp(topic, "wohnzimmer/essen/brightness") == 0)|| (strcmp(topic, "wohnzimmer/essen/bstatus") == 0))
  {
    int p;
      float fp=0;
      sscanf(payload, "%f", &fp);
      p = fp;
    if((pUpdate2+2000)< now) // only update current power if button was not turned for at least two seconds, otherwise this is probably our own message with old power values
    {
        oldp2 = powerEssen = p;
        
      lv_bar_set_value(ui_Tischslider,powerEssen,LV_ANIM_ON);
    }
  }
  if ((strcmp(topic, "wohnzimmer/essen/command") == 0) || (strcmp(topic, "wohnzimmer/essen/status") == 0))
  {
    if ((pUpdate2 + 2000) < now) // only update current power if button was not turned for at least two seconds, otherwise this is probably our own message with old power values
    {
      
      if (strcmp(payload, "ON") == 0)
      {
        lightState2 = true;
        lv_obj_add_state(ui_TischToggle, LV_STATE_CHECKED);
      }
      else
      {
        lightState2 = false;
        lv_obj_clear_state(ui_TischToggle, LV_STATE_CHECKED);
      }
     // DebugPrintf("lightState2In%d\n", (int)lightState2);
    }
  }
  if ((strcmp(topic, "wohnzimmer/wohnen/command") == 0) || (strcmp(topic, "wohnzimmer/wohnen/status") == 0))
  {
    if ((pUpdate + 2000) < now) // only update current power if button was not turned for at least two seconds, otherwise this is probably our own message with old power values
    {
      if (strcmp(payload, "ON") == 0)
      {
        lightState1 = true;
        lv_obj_add_state(ui_SofaToggle, LV_STATE_CHECKED);
      }
      else
      {
        lightState1 = false;
        lv_obj_clear_state(ui_SofaToggle, LV_STATE_CHECKED);
      }
      //DebugPrintf("lightState1%d\n", (int)lightState1);
    }
  }
  if ((strcmp(topic, "wohnzimmer/wohnen/brightness") == 0)|| (strcmp(topic, "wohnzimmer/wohnen/bstatus") == 0))
  {
    int p;
      float fp=0;
      //sscanf(payload, "0,0,%d", &p);
      sscanf(payload, "%f", &fp);
      p = fp;
    if((pUpdate+2000)< now) // only update current power if button was not turned for at least two seconds, otherwise this is probably our own message with old power values
    {
        oldp1 = powerWohnen = p;
      lv_bar_set_value(ui_SofaSlider,powerWohnen,LV_ANIM_ON);
    }
  }

  if (strcmp(topic, "PV/Battery/level") == 0)
  {
    sscanf(payload,"%d",&batteryLevel);
  }
  else if (strcmp(topic, "Electricity/currentPrice") == 0)
  {
    sscanf(payload,"%f",&price);
  }
  else if (strcmp(topic, "PV/Inverter/Production") == 0) //[W]
  {
    sscanf(payload,"%f",&production);
    updateDisplay();
  }
  else if (strcmp(topic, "PV/Inverter/power") == 0)//[kW]
  {
    sscanf(payload,"%f",&power);
    updateDisplay();
  }
  else if (strcmp(topic, "PV/Gridconsumption") == 0)//[W]
  {
    sscanf(payload,"%f",&einspeisung);
    updateDisplay();
  }
  
  else if (strcmp(topic, "IOT/Ping") == 0)
  {
    networkTimeout = 0;
    PingArrived = true;
  }
  }

void sendState()
{
  
  char buf[50];
  sprintf(buf, "%d", batteryLevel);
  client.publish("PV/Battery/myLevel", buf);
}

void reconnect()
{
#ifdef NO_MQTT
  return;
#endif

  DebugPrintf("Attempting MQTT connection...\n");
  // Attempt to connect
  if (client.connect("PVDisplayTouch"))
  {
    DebugPrintf("MQTTconnected\n");
    // Once connected, publish an announcement...
    sendState();
    // ... and resubscribe
    client.subscribe("PV/Battery/level");
    client.subscribe("PV/Inverter/Production"); //[W]
    client.subscribe("PV/Inverter/power"); //[kW]
    client.subscribe("PV/Gridconsumption"); //[W]
    client.subscribe("Electricity/currentPrice");
    client.subscribe("wohnzimmer/essen/command");
    client.subscribe("wohnzimmer/essen/brightness");
    client.subscribe("wohnzimmer/wohnen/command");
    client.subscribe("wohnzimmer/wohnen/brightness");
    client.subscribe("wohnzimmer/essen/status");
    client.subscribe("wohnzimmer/essen/bstatus");
    client.subscribe("wohnzimmer/wohnen/status");
    client.subscribe("wohnzimmer/wohnen/bstatus");
    client.subscribe("IOT/Ping");
  }
  else
  {
    DebugPrintf("failed, rc=");
    DebugPrintf("%d", client.state());
    DebugPrintf(" try again in 5 seconds\n");
  }
}
static float itOld = 0;
static float stOld = 0;
static float ihOld = 0;
static bool oldHeating = false;
static bool oldIsOn = false;

void reconnectWifi()
{
  bool ledState = false;
  while (WiFi.status() != WL_CONNECTED)
  {
    long start = millis();
    while (millis() - start < 500)
    {
      localLoop();
    }
    ledState = !ledState;
  }
}
long lastReconnectAttempt = 0;
long lastReconnectWifiAttempt = 0;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("SerialBegin");
    Serial.printf("SRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.printf("PSRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  Serial.println("initTouch");

  // Init touch device
  touch_init();
  WiFi.persistent(false);
  Serial.println("afterPersistent");
  WiFi.mode(WIFI_STA);
  Serial.println("aftermode(WIFI_STA");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("afterWiFi.begin");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(200);
    //lv_timer_handler();
  }

  // Init Display
  gfx->begin(11000000);
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  ledcSetup(0, 600, 8);
  ledcAttachPin(GFX_BL, 0);
  //ledcWrite(0, 150);
  ledcWrite(0, 150);
#endif

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();
  lv_color_t *buf_3_1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); //1/10th should be enough but we have plenty
  //lv_color_t *buf_3_2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  //if (!buf_3_1 && !buf_3_2) // 
  if (!buf_3_1)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
     lv_disp_draw_buf_init(&draw_buf, buf_3_1, NULL, screenWidth * screenHeight);
     //lv_disp_draw_buf_init(&draw_buf, buf_3_1, buf_3_2, screenWidth * screenHeight);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);


    Serial.printf("SRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.printf("PSRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  }

  lv_obj_clean(lv_scr_act());
  ui_init();
  lv_disp_load_scr(ui_Init); 
  lv_timer_handler();
  //pinMode(pButton1, INPUT_PULLUP);
  DebugPrintf("%s\n", WiFi.localIP().toString().c_str());
  lv_label_set_text(ui_IPLabel, WiFi.localIP().toString().c_str());
  lv_timer_handler();

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("pvdisplay");
  ArduinoOTA.onStart([]()
                     {    
                      //lv_disp_load_scr(ui_OTA);    
                      });
  ArduinoOTA.onEnd([]()
                   {
                      //lv_disp_load_scr(ui_PV);  
                   });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
  esp_task_wdt_reset(); // reset the watchdog
                         /* int pnum = (progress / (total / 100));
                          static int oldPnum=0;
                          if(pnum>oldPnum+1)
                          {
                            oldPnum = pnum;
                          lv_bar_set_value(ui_FirmwareBar, pnum, LV_ANIM_ON);
                          }*/
                         //lv_timer_handler();
                        });

  ArduinoOTA.onError([](ota_error_t error)
                     {
                       DebugPrintf("Error[ %u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         DebugPrintf("Auth Failed\n");
                       else if (error == OTA_BEGIN_ERROR)
                         DebugPrintf("Begin Failed\n");
                       else if (error == OTA_CONNECT_ERROR)
                         DebugPrintf("Connect Failed\n");
                       else if (error == OTA_RECEIVE_ERROR)
                         DebugPrintf("Receive Failed\n");
                       else if (error == OTA_END_ERROR)
                         DebugPrintf("End Failed\n");
                     });
  ArduinoOTA.begin();

  DebugPrintf("%s\n", WiFi.localIP().toString().c_str());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  DebugPrintf("anotherIPPrint%s\n", WiFi.localIP().toString().c_str());
  esp_task_wdt_init(25, true); //socket timeout is 15seconds
  esp_task_wdt_add(nullptr);

  DebugPrintf("watchdog%s\n", WiFi.localIP().toString().c_str());
 // button1.init(false);

  
  lv_disp_load_scr(ui_PV); 
  DebugPrintf("loadScreen%s\n", WiFi.localIP().toString().c_str());
  
  lv_bar_set_value(ui_SofaSlider,powerWohnen,LV_ANIM_ON);
  lv_bar_set_value(ui_Tischslider,powerEssen,LV_ANIM_ON);
  pUpdate = millis();
  pUpdate2 = millis();
}

void loop() {



  if (WiFi.status() != WL_CONNECTED)
  {
    if (now - lastReconnectWifiAttempt > 60000) // every 60 seconds
    {
      lastReconnectWifiAttempt = now;
      // Attempt to reconnect
      reconnectWifi();
    }
  }
  if (!client.connected())
  {
    if (now - lastReconnectAttempt > 10000) // every 10 seconds
    {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      reconnect();
    }
  }
  else
  {
    // Client connected

    client.loop();
  }

  localLoop(); // local updates which have to be done during reconnect as well;
  static int32_t oldBatteryLevel = 0;
  if (batteryLevel != oldBatteryLevel)
  {
    oldBatteryLevel = batteryLevel;
    updateDisplay();
  }
  static float oldPrice=0.0;
  if(price != oldPrice)
  {
    updateDisplay();
    oldPrice = price;
  }
}


void localLoop()
{
  //debounceButton::update();
  esp_task_wdt_reset(); // reset the watchdog
  ArduinoOTA.handle();
  now = millis();
  lv_timer_handler();
  static int oldx=0;
  static int oldy=0;
  if(oldx!=touch_last_x || oldy != touch_last_y)
  {
    oldx = touch_last_x;
    oldy = touch_last_y;
    /*gfx->writeFillRectPreclipped(0,0,oldx,oldy,0xffff);
    gfx->flush();*/
    DebugPrintf("%d %d %d %d\n",oldx,oldy,screenWidth,screenHeight);
  }
  bool currentState2 = lv_obj_has_state(ui_TischToggle, LV_STATE_CHECKED);
  if (currentState2 != lightState2)
  {
    lightState2 = currentState2;
    if (lightState2)
      client.publish("wohnzimmer/essen/command", "ON");
    else
      client.publish("wohnzimmer/essen/command", "OFF");
    pUpdate2 = now;
      DebugPrintf("UpdateTime3Essen\n");
  }
  bool currentState1 = lv_obj_has_state(ui_SofaToggle, LV_STATE_CHECKED);
  if (currentState1 != lightState1)
  {
    lightState1 = currentState1;
    if (lightState1)
      client.publish("wohnzimmer/wohnen/command", "ON");
    else
      client.publish("wohnzimmer/wohnen/command", "OFF");
    pUpdate = now;
      DebugPrintf("UpdateTime4Sofa\n");
  }
  static unsigned long lastchange1 = 0;
  static unsigned long lastchange2 = 0;
    if((now - lastchange1 > 500) && powerWohnen!=lv_bar_get_value(ui_SofaSlider))
    {
      lastchange1 = now;
      powerWohnen = lv_bar_get_value(ui_SofaSlider);
      DebugPrintf("p1 %d\n",powerWohnen);
      if(powerWohnen == 0)
      {
          client.publish("wohnzimmer/wohnen/command", "OFF");
          lv_obj_clear_state(ui_SofaToggle, LV_STATE_CHECKED);
          lightState1 = false;
      }
      else if(lightState1 == false)
      {
        
          client.publish("wohnzimmer/wohnen/command", "ON");
          lv_obj_add_state(ui_SofaToggle, LV_STATE_CHECKED);
          lightState1 = true;
      }
          
      char val[50];
      sprintf(val, "%d", powerWohnen);
      client.publish("wohnzimmer/wohnen/brightness", val);
      pUpdate = now;
      DebugPrintf("UpdateTime1bSofa\n");
    }
    if((now - lastchange2 > 500) && powerEssen!=lv_bar_get_value(ui_Tischslider))
    {
      lastchange2 = now;
      powerEssen = lv_bar_get_value(ui_Tischslider);
      DebugPrintf("p2 %d\n",powerEssen);
      if(powerEssen == 0)
      {
          client.publish("wohnzimmer/essen/command", "OFF");
          lv_obj_clear_state(ui_TischToggle, LV_STATE_CHECKED);
          lightState2 = false;
      }
      else if(lightState2 == false)
      {
        
          client.publish("wohnzimmer/essen/command", "ON");
          lv_obj_add_state(ui_TischToggle, LV_STATE_CHECKED);
          lightState2 = true;
      }
      char val[50];
      sprintf(val, "%d", powerEssen);
      client.publish("wohnzimmer/essen/brightness", val);
      pUpdate2 = now;
      DebugPrintf("UpdateTime2Bessen\n");
    }
/*
  if (button1.wasPressed())
  {
    ESP.restart();
  }*/

  static unsigned long networkMinute = 0;
  if ((now - networkMinute) > 60000)
  {
    networkMinute = now;
    if (PingArrived) // only activate network timeout if at least one Ping has arrived
    {
      networkTimeout++; // this is reset whenever an mqtt network ping arrives
    }
  }
  if (networkTimeout > 5) // 5 minute timeout
  {
    DebugPrintf("network Timeout %d\n", networkTimeout);
    ESP.restart();
  }
}

void updateDisplay()
{
  
    char text_buffer[128];
    if(batteryLevel==0)
    {
      strcpy(text_buffer,"invalid");
    }
    else
    {
    sprintf(text_buffer, "%d%%", batteryLevel);
    }
    lv_bar_set_value(ui_Batterie, batteryLevel, LV_ANIM_OFF);
    lv_label_set_text(ui_Batteriewert, text_buffer);
    static int32_t oldBL = 0;
    if(oldBL < batteryLevel)
    {    
      oldBL = batteryLevel;
      lv_obj_set_style_bg_color(ui_Batterie, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else if(oldBL > batteryLevel)
    {
      oldBL = batteryLevel;
      lv_obj_set_style_bg_color(ui_Batterie, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    sprintf(text_buffer, "%3.1f ct", price * 100.0);
    if (price > 0.3)
    {
      lv_obj_set_style_text_color(ui_PreisLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_text_color(ui_Preis, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else if (price < 0.1)
    {
      lv_obj_set_style_text_color(ui_PreisLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_text_color(ui_Preis, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      lv_obj_set_style_text_color(ui_PreisLabel, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_text_color(ui_Preis, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    lv_label_set_text(ui_Preis, text_buffer);

    if (production > 1000)
      sprintf(text_buffer, "%3.3f\nkW", production / 1000.0);
    else
      sprintf(text_buffer, "%d\nW", (int)production);
    lv_label_set_text(ui_PV1, text_buffer);

    if (production > 1000)
      sprintf(text_buffer, "%3.3f\nkW", production / 1000.0);
    else
      sprintf(text_buffer, "%d\nW", (int)production);
    lv_label_set_text(ui_PV1, text_buffer);
    float verbrauch = power * 1000.0 - einspeisung;
    float Akkuladeleistung = production - einspeisung - verbrauch;
    if (Akkuladeleistung > 1000.0 || Akkuladeleistung < -1000.0)
    {
      sprintf(text_buffer, "%3.3f\nkW", fabs(Akkuladeleistung / 1000.0));
    }
    else
      sprintf(text_buffer, "%d\nW", abs((int)(Akkuladeleistung)));
    lv_label_set_text(ui_Akku1, text_buffer);
    if (Akkuladeleistung > 0)
    {
      lv_obj_set_style_border_color(ui_AkkuRing, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      lv_obj_set_style_border_color(ui_AkkuRing, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    if (verbrauch > 1000 || verbrauch < -1000)
      sprintf(text_buffer, "%3.3f\nkW", verbrauch / 1000.0);
    else
    sprintf(text_buffer, "%d\nW", (int)(verbrauch) );
    lv_label_set_text(ui_Verbrauch1, text_buffer);

    if(einspeisung > 1000 || einspeisung < -1000)
    sprintf(text_buffer, "%3.3f\nkW", einspeisung/-1000.0);
    else
    sprintf(text_buffer, "%d\nW", (int)(einspeisung)*-1 );
    lv_label_set_text(ui_Netz1, text_buffer);
    if (einspeisung < -10)
    {
      //smartdisplay_set_led_color(lv_color32_t({.ch = {.blue = 0, .green = 0, .red = 255}}));
      lv_obj_set_style_border_color(ui_NetzRing, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      //smartdisplay_set_led_color(lv_color32_t({.ch = {.blue = 0, .green = 255, .red = 0}}));
      lv_obj_set_style_border_color(ui_NetzRing, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}
