#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

            // заносим параметры WiFi сети 
const char* ssid = "Door";
const char* password = "12345678";
                //* Настройка IP-адреса */
IPAddress local_ip(192,168,10,1);
IPAddress gateway(192,168,10,1);
IPAddress subnet(255,255,255,0);

bool regimConfig = false; // переключает в режим ввода лиц для распознавания
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// определите одну из возможных моделей камеры
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;


#define relay_pin 12 // контакт микросхемы, управляющий реле
unsigned long door_opened_millis = 0; // переменная, которая временно хранит число милисекунд
long interval = 1000;           // время открытия двери (длительность импульса реле)
bool face_recognised = false;

void app_facenet_main();
void app_httpserver_init();

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

//****************************************************************
static dl_matrix3du_t *image_matrix = NULL;
static box_array_t *detected_face1 = NULL;
static dl_matrix3d_t *face_id1 = NULL;
//****************************************************************

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

// Функция обработки прерывания по пину
static void IRAM_ATTR detectsMovement(void *arg) {
  Serial.println("Regim Config!");
  regimConfig = true;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();



  digitalWrite(relay_pin, LOW);
  pinMode(relay_pin, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(4, OUTPUT);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //инициализация с высокими характеристиками для предварительного выделения больших буферов
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE) // доп. настройки для этой модели камеры
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Настраиваем камеру в соответствии со структурой config
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

// Настраиваем контакт для переключения в режим ввода лиц (через прерывание)              
  pinMode(GPIO_NUM_15, INPUT_PULLUP);                                                                             
  // Установите PIN в качестве прерывания, назначьте функцию прерывания и установите режим НАРАСТАНИЯ 
  esp_err_t err1;
  err1 = gpio_isr_handler_add(GPIO_NUM_15, &detectsMovement, (void *) 15);
  if (err1 != ESP_OK) {
    Serial.printf("обработчик добавления не удалось с ошибкой 0x%x \r\n", err1);
  }
  err1 = gpio_set_intr_type(GPIO_NUM_15, GPIO_INTR_POSEDGE);
  if (err1 != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err1);
  }

  // Настраиваем WiFi как точку доступа ********************************************************** 
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();

  app_httpserver_init();
  app_facenet_main();
  image_matrix = dl_matrix3du_alloc(1, 320, 240, 3); // выделяем память для структуры, содержащей растровое изображением для распознавания лиц **********
  socket_server.listen(82);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(myIP);
  Serial.println("' to connect");
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); //send face to browser
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

void open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH); //close (energise) relay so door unlocks
    Serial.println("Door Unlocked");
    client.send("door_open");
    door_opened_millis = millis(); // time relay closed and door opened
  }
}

void loop() 
{
  if(regimConfig) // если включен режим занесения лиц для распознавания
  {  
    WiFi.mode(WIFI_AP);   // включаем WiFi
    auto client = socket_server.accept();
    client.onMessage(handle_message);
    send_face_list(client);
    client.send("STREAMING"); // по умолчанию режим STREAMING (просмотр)
    while (client.available())
    {
      client.poll();
      if (millis() - interval > door_opened_millis) // если дверь открыта, то
      {
        digitalWrite(relay_pin, LOW); //закроем её
      }
      fb = esp_camera_fb_get(); // получаем ссылку на снимок
      if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
      {
        fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
        detected_face1 = face_detect(image_matrix, &mtmn_config);
        if (detected_face1) // если обнаружено лицо
          {
            if (align_face(detected_face1, image_matrix, aligned_face) == ESP_OK) // и удалось его обработать для распознавания
            {
              face_id1 = get_face_id(aligned_face);
              last_detected_millis = millis();
              if (g_state == START_DETECT)  // если включен режим только обнаружения лица
              {
                client.send("FACE DETECTED");
              }
                if (g_state == START_ENROLL)  // если включен режим запоминания лица
                {
                  int left_sample_face = do_enrollment(&st_face_list, face_id1);
                  char enrolling_message[64];
                  sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
                  client.send(enrolling_message);
                  if (left_sample_face == 0)
                  {
                    ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
                    g_state = START_STREAM;
                    char captured_message[64];
                    sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
                    client.send(captured_message);
                    send_face_list(client);
                  }
                }
                if (g_state == START_RECOGNITION  && (st_face_list.count > 0))  // если включено распознавание лиц и список лиц не пуст
                {
                  face_id_node *f = recognize_face_with_name(&st_face_list, face_id1);
                  if (f)
                  {
                    char recognised_message[64];
                    sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
                    open_door(client);
                    client.send(recognised_message);
                  }
                  else
                  {
                    client.send("FACE NOT RECOGNISED");
                  }
                }
              dl_matrix3d_free(face_id1);
            }
          }
          else
          {
            if (g_state != START_DETECT)  // если включен режим распознавания или запоминация лица, а лицо не обнаружено
            {
              client.send("NO FACE DETECTED");
            }
          }
        if (g_state == START_DETECT && millis() - last_detected_millis > 500) // если включен режим обнаружения лица и закончилось время обнаружения - 500 милисекунд)
        {
          client.send("DETECTING");
        }
      }
      client.sendBinary((const char *)fb->buf, fb->len);  // отправляем снимок на браузер
      esp_camera_fb_return(fb);   // очищаем буфер для следующего снимка
      fb = NULL;
    }
  }
  else  //********** если включен рабочий режим ************
  {  
    WiFi.mode(WIFI_OFF);  // отключаем WiFi
    if (millis() - door_opened_millis > interval) // если прошла секунда (interval = 1000 милисекунд)
      { 
        digitalWrite(relay_pin, LOW); // отключаем реле
      }
    //digitalWrite(4,HIGH);//************ для распознавания лица, включаем подсветку
    //delay(150);
    fb = esp_camera_fb_get(); // Получаем указатель на снимок
    //digitalWrite(4,LOW);  // отключаем подсветку
    fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item); //преобразовываем снимок для поиска лица
    detected_face1 = face_detect(image_matrix, &mtmn_config); // ищем лицо
      if (detected_face1) // если лицо обнаружено
      {
        Serial.println("Face detect");
        if (align_face(detected_face1, image_matrix, aligned_face) == ESP_OK)
        {
          face_id1 = get_face_id(aligned_face); // преобразовываем обнаруженное лицо для распознавания
            if ( st_face_list.count > 0)  // если список лиц не пуст
            {
              face_id_node *f = recognize_face_with_name(&st_face_list, face_id1);  // ищем лицо в списке лиц
              if (f)    // если лицо есть в списке, открываем дверь
              {
                if (digitalRead(relay_pin) == LOW) 
                {
                  digitalWrite(relay_pin, HIGH); // включаем реле, чтобы открыть дверь
                  Serial.println("Door Unlocked");
                  door_opened_millis = millis(); // начинаем отсчет времени работы реле
                }
              }  
            }
          dl_matrix3d_free(face_id1); // очищаем память
        }
      }
    esp_camera_fb_return(fb); // очищаем буфер для следующего снимка
    fb = NULL;
  }
}
