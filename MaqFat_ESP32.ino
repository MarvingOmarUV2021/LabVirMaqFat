  /**************************************************************************
 * 
 * DIPLOMADO: Internet de las Cosas 
 * Samsung Innovation Campus
 * Código IoT
 * 
 * PROFESORES:
 * Hugo Vargas
 * Paloma Vilchis
 * 
 * PROYECTO CAPSTON
 * Supervisión y accionamiento de una máquina de ensayos 
 * de fatiga mediante instrumentación virtual en la 
 * educación no presencial.
 * 
 * EQUIPO #12 
 * M.C. Jesús Medina Cervantes
 * Dr. Edgar Mejía Sánchez
 * Dr. Marving Omar Aguilar Justo
 * 
 * Fecha: 4/agosto/2022.
 * 
 */

// Bibliotecas

#include <SPI.h>              // Biblioteca - Comunicación disps. SPI (sensor HX711)
#include <Wire.h>             // Biblioteca - Comunicación disps. bus I2C
#include <Adafruit_GFX.h>     // Biblioteca - Pantalla OLED 
#include <Adafruit_SSD1306.h> // Biblioteca - Pantalla OLED
#include <WiFi.h>             // Biblioteca - Conexión Wifi
#include "HX711.h"            // Biblioteca - Celda de carga HX711
#include <PubSubClient.h>     // Biblioteca - Conexión MQTT


// Parámetros asociados al WiFi

const char* ssid = "jesusmc";   // Nombre de red-wifi en celular
const char* password = "1234";  // Contraseña de red-wifi por celular
WiFiClient espClient;           // Maneja los datos de conexion WiFi


// Parámetros asociados al broker MQTT

const char* mqtt_server = "192.168.43.111";   // IP asignada red-wifi por celular
IPAddress server(192,168,43,111);             // IP asignada red-wifi por celular
PubSubClient client(espClient);               // Maneja los datos de conexion al broker


// Parámetros asociados a las interrupciones

hw_timer_t * timer = NULL;                            // Asigna un valor nulo al temporizador
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;    // Gestiona las interrupciones externas
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Gestiona la interrupciòn interna


// Parámetros asociados al OLED Display

#define I2C_SDA          21   // Conexión con el ESP32: GPIO21(SDA)
#define I2C_SCL          22   // Conexión con el ESP32: GPIO22(SCL)
#define SCREEN_WIDTH    128   // OLED display ancho en pixeles
#define SCREEN_HEIGHT    64   // OLED display alto en pixeles
#define OLED_RESET       -1   // Reset pin # (Usar -1 si se comparte el Reset pin del ESP32)
#define SCREEN_ADDRESS 0x3C   // 0x3C para OLED display de 128x64 pixeles

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Parámetros asociados al LED empotrado al ESP32

#define LED_BUILTIN 2           // Led del ESP32
volatile int ledState = LOW;    // Led del ESP32 apagado


// Parámetros asociados a la Fuerza, a las Vueltas y a la Velocidad 

HX711 scale;                  // Variable para calibrar más adelante la CdC
char valor[10];               // Para imprimir en el OLED el valor de la CdC HX711 
char valvueltas[10];          // Para imprimir en el OLED el valor de las vueltas del motor
float lectura;                // lectura actual de la CdC
float lectura_anterior = 0.0; // lectura anterior de la CdC
volatile int frecuencia;      // velocidad del motor en ciclos por segundo (rps)
uint32_t vueltas_total;       // registrar la cant. total de vueltas del motor cuando finalice la prueba
volatile int8_t bandera = 0;  // bandera (=1) para asegurar que "se congele" el programa cuando la fuerza f < -5
volatile int8_t estado = 0;   // estado (=1) para indicar la finalización de la prueba 


// Parámetros asociados a la señal PWM 

const int freq = 10000;           // frecuencia (Hz) de la PWM hacia el variador
const int ledChannel = 0;         // canal 0 por donde estará la PWM (pudo ser del 0 al 15)
const int resolution = 8;         // Resol. de 8 bits para controlar el voltaje con valores de 0 a 255 (pudo ser de 1 a 16 bits)
uint8_t dutyCycle;                // Var. del ancho de pulso de la PWM


// Parámetros varios

volatile uint32_t vueltas =  0;   // variable para contar las vueltas del motor
volatile uint32_t vueltast = 0;   // variable para registrar la cant. total de vueltas del motor
unsigned long timeNow, timeLast;  // Variables para el control de tiempo no bloqueante
int dato = 0;                     // Contador
int wait = 2000;                  // Indica la espera cada 2 segundos para envío de mensajes MQTT
char dataString[8];               // Define una arreglo de 8 caracteres para enviarlos por MQTT 
String texto;                     // Var. para almacenar la info. (fuerza, vel, vueltas y estado) en formato JSON
char datos[80];                   // Define una arreglo de 80 caracteres para la variable "texto"


// Definición de pines

const int LOADCELL_DOUT_PIN = 19; // GPIO19 ESP32 (SPI MISO)
const int LOADCELL_SCK_PIN =  18; // GPIO18 ESP32 (SPI SCK)

#define BTN_RESET    35           // Pin del ESP32 para reiniciar el ESP32
#define MOTOR_INICIO 17           // Pin del ESP32 para enviar señal de ARRANQUE del motor
#define PIN_PWM      16           // Pin del ESP32 para enviar ajuste de frecuencia del variador
#define MOTOR_PARO   33           // Pin del ESP32 para enviar señal de PARO del motor
#define CNY70        34           // Pin del ESP32 que recibe un pulso digital cada vuelta del motor


// ----------------------------------------------------------------------------------- //
// --------------               Funciones de interrupciones             -------------- //
// ----------------------------------------------------------------------------------- //

void IRAM_ATTR isr() {          // Sensor CNY70 cuenta 1 vuelta (Interrup. Externa)
  portENTER_CRITICAL(&synch);   // Protege que no entre otra interrupción mientras esta se ejecuta
  vueltas ++;                   // Se incrementa en 1 el número de vueltas
  vueltast ++;                  // Se incrementa en 1 el número de vueltas totales
  portEXIT_CRITICAL(&synch);    // Se finaliza la protección de la interrupción
}

void IRAM_ATTR isr_reset() {    // Poner a cero el no. de vueltas para iniciar prueba (Interrup. Externa)
  portENTER_CRITICAL(&synch);   // Protege que no entre otra interrupción mientras esta se ejecuta
  vueltas = 0;                  // Se reinicia el conteo de vueltas
  vueltast = 0;                 // Se reinicia el conteo de vueltas totales
  bandera = 0;                  // bandera (=1) para asegurar que "se congele" el programa cuando la fuerza f < -5
  estado = 0;                   // estado (=1) para indicar la finalización de la prueba 
  portEXIT_CRITICAL(&synch);    // Se finaliza la protección de la interrupción
}

void IRAM_ATTR onTimer() {              // Temporizador (Interrupción interna cada 1 segs)
  portENTER_CRITICAL_ISR(&timerMux);    // Protege que no entre otra interrupción mientras esta se ejecuta
  frecuencia = vueltas;                 // "frecuencia" recibe el valor de las vueltas dadas por cada segundo                                  
  vueltas = 0;                          // se reinicia la variable "vueltas"
  ledState = !ledState;                 // se cambia el estado de "ledState" (si era alto cambia a bajo, y viceversa)
  digitalWrite(LED_BUILTIN, ledState);  // el LED del ESP32 parpadea cada 1 segs
  portEXIT_CRITICAL_ISR(&timerMux);     // Se finaliza la protección de la interrupción
}


// ----------------------------------------------------------------------------------- //
// --------------                        VOID SETUP                     -------------- //
// ----------------------------------------------------------------------------------- //

void setup() {
  
  delay(2000);
  
  pinMode(LED_BUILTIN, OUTPUT);         // Configurar pin 2 (Led del ESP32) como salida
  digitalWrite(LED_BUILTIN, ledState);  // Apagar el Led del ESP32
  
  pinMode(CNY70, INPUT);                // Configurar pin 34 (Sensor de vueltas) como entrada
  attachInterrupt(CNY70, isr, RISING);  // Interrupción cuando el pin 34 recibe señal con flanco LOW/HIGH

  pinMode(BTN_RESET, INPUT);            // Configurar pin 35 (botón reset ESP32) como entrada
  attachInterrupt(BTN_RESET, isr_reset, RISING);  // Interrupción cuando el pin 35 recibe señal con flanco LOW/HIGH

  pinMode(MOTOR_PARO, OUTPUT);          // Configurar pin 33 (señal de paro del motor) como salida
  digitalWrite(MOTOR_PARO, HIGH);       // Asignar un estado "alto" para tener al paro inactivo.

  pinMode(MOTOR_INICIO, OUTPUT);        // Configurar pin 17 (señal de arranque del motor) como salida
  digitalWrite(MOTOR_INICIO, LOW);      // Asignar un estado "bajo" para tener al arranque inactivo.

  ledcSetup(ledChannel,freq,resolution);// Configuración de las funcionalidades del PIN_PWM
  ledcAttachPin(PIN_PWM, ledChannel);   // Asigna el canal 0 al GPIO 16
  dutyCycle = 0;                        // De inicio se declara un cicloo de trabajo igual a 0
  ledcWrite(ledChannel, dutyCycle);     // Coloca la PWM en el GPIO 16 mediante el canal 0


  // Ajuste a cero de la celda de carga
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // Inicia el uso de la biblioteca
  scale.set_scale(112230.769f);                     // Divisor de la CdC (escala usada para calibrar la CdC a 20 kg)
  scale.tare();                                     // Reinicia la escala a 0

  
  // Rutina para detener el programa si el display no tiene su voltaje de operación igual a 3.3 Volts
  // SSD1306_SWITCHCAPVCC = genera el voltage interno de 3.3V para el display 
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Se detiene el programa mediante ciclo infinito
  }


  // Muestra en el display el mensaje de inicio

  display.clearDisplay();               // Limpieza del display
  display.setTextSize(2);               // Tamaño del texto
  display.setCursor(0, 10);             // Localización del texto
  display.setTextColor(SSD1306_WHITE);  // Color del texto (blanco)
  display.println("Prueba de Fatiga");  // Mensaje a imprimir en el display
  display.display();                    // Envío del mensaje al display 
  delay(2000);                          // Espera bloqueante de 2 segundos
  display.clearDisplay();               // Limpieza del display
  
  
  // Muestra en el display las variables fuerza, velocidad y vueltas sin valores aún
  
  display.setTextSize(1);               // Tamaño del texto
  display.setTextColor(SSD1306_WHITE);  // Color del texto (blanco)
  display.setCursor(40, 0);             // Localización del texto
  display.println("Fuerza:");           // Mensaje a imprimir en el display
  display.setCursor(30, 25);            // Localización del texto
  display.println("Velocidad:");        // Mensaje a imprimir en el display
  display.setCursor(35, 45);            // Localización del texto
  display.println("Vueltas:");          // Mensaje a imprimir en el display
  display.display();                    // Envío de los 3 mensajes al display 

 
  // Configuración del temporizador (interrupción interna)
  //  - El temporizador se genera con 2000 ciclos por segundo, que 
  //    es la división de 80 MHz por 40000.
  //  - La interrupción es cada segundo, resultado de dividir 
  //    2000 por 2000.
  //  - El contador del "timer" se reinicia cuando se activa la 
  //    interrupción.
   
  timer = timerBegin(0, 40000, true);           // Se genera el temporizador "timer"; el conteo es ascendente (true)
  timerAttachInterrupt(timer, &onTimer, true);  // Asigna el temporizador a la interrupción "&onTimer"; la acción es en el flanco (true)
  timerAlarmWrite(timer, 2000, true);           // Se configura la frec. de la interrupción (cada 1 seg); se reinicia el temporizador (true)
  timerAlarmEnable(timer);                      // Se inicia el conteo del temporizador
  
  
  // Configuración del monitor serial
  
  Serial.begin (115200);        // Se inicia comunicación serial
  Serial.println();             // Salto de renglón en el monitor serial
  Serial.println();             // Salto de renglón en el monitor serial
  Serial.print("Conectar a ");  // Impresión de mensaje en el monitor serial
  Serial.println(ssid);         // Impresión de red-WiFi en el monitor serial
  
  
  // Configuración de la conexión WiFi

  WiFi.begin(ssid, password);   // Esta es la función que realiza la conexión a WiFi

  while (WiFi.status() != WL_CONNECTED) {   // Este bucle espera a que se realice la conexión
    delay(500);                             // se usa espera bloqueante
    Serial.print(".");                      // Indicador de progreso
  }
  
  Serial.println();                   // Salto de renglón en el monitor serial
  Serial.println("WiFi conectado");   // Impresión de mensaje en el monitor serial
  Serial.println("Direccion IP: ");   // Impresión de mensaje en el monitor serial
  Serial.println(WiFi.localIP());     // Impresión de IP en el monitor serial

  delay (1000);     // Espera bloqueante antes de iniciar la comunicación con el broker


  // Conexión con el broker MQTT

  client.setServer(mqtt_server, 1883);  // Conectarse a la IP del broker en el puerto 1883
  client.setCallback(callback);         // Esta función permite recibir mensajes MQTT
  delay(1500);                          // Espera preventiva (para no perder información)

  timeLast = millis ();   // Inicia el control de tiempo
   
}

// ----------------------------------------------------------------------------------- //
// --------------                        VOID LOOP                      -------------- //
// ----------------------------------------------------------------------------------- //

void loop() {

  if (!client.connected()) {    // Verificar siempre que haya conexión al broker
    reconnect();                // En caso de que no haya conexión, ejecutar la función de reconexión
  }
  client.loop();                // Se ejecuta de manera no bloqueante las funciones necesarias para la 
                                // comunicación con el broker
  lectura = scale.get_units(5); // "lectura" es el promedio de 5 lecturas del ADC

  // Ante el ensayo de fatiga, la fuerza en el material bajo prueba va aumentando, pero llega un momento en que 
  // el material se rompe, en ese momento la fuerza baja abruptamente, y en ese momento, la diferencia en la 
  // condicional se torna negativa.
  if ((lectura - lectura_anterior) < -5.0){ // Cuando se cumpla la condición se termina la prueba.
    vueltas_total = vueltast;               // Se registra la cantidad de vueltas que hubo hasta que terminó la prueba
    digitalWrite(MOTOR_PARO, LOW);          // Se envía una señal para abrir el interruptor que para al motor. 
    delay(100);                             // Espera bloqueante de 100 ms
    digitalWrite(MOTOR_PARO, HIGH);         // Como ya se paró el motor, se puede reiniciar el estado a "HIGH"
    estado = 1;                             // Es una bandera que se enviará como código JAVA más adelante

    display.fillRect(40,10,60,8,SSD1306_BLACK);   // Para mostrar en la pantalla OLED una gráfica tipo barra   
    display.setCursor(40, 10);                    // Ubicación del texto que se enviará a la pantalla OLED
    sprintf(valor,"%.2f kgf", lectura);           // La cantidad de fuerza se convierte en char
    display.println(valor);                       // La fuerza en kgf que se enviará a la pantalla OLED
    display.fillRect(40,35,60,8,SSD1306_BLACK);   // Para mostrar en la pantalla OLED una gráfica tipo barra   
    sprintf(valvueltas,"%d RPS", frecuencia);     // La cantidad de velocidad se convierte en char
    display.setCursor(40, 35);                    // Ubicación del texto que se enviará a la pantalla OLED
    display.println(valvueltas);                  // La velocidad en rps que se enviará a la pantalla OLED
    display.fillRect(20,55,110,8,SSD1306_BLACK);  // Para mostrar en la pantalla OLED una gráfica tipo barra   
    sprintf(valor,"%u", vueltas_total);           // La cantidad de vueltas totales se convierte en char
    display.setCursor(40, 55);                    // Ubicación del texto que se enviará a la pantalla OLED
    display.println(valor);                       // El total de vueltas que se enviará a la pantalla OLED
    display.setCursor(0, 55);                     // Ubicación del texto que se enviará a la pantalla OLED
    display.println("FIN");                       // Texto que se enviará a la pantalla OLED
    display.display();                            // Envío de los 3 mensajes al display

    
    
    // Almacenamiento de información en formato JSON (como arreglo de 80 caracteres)
    // 1. La fuerza sensada por la Celda de Carga
    // 2. La velocidad del motor en rps
    // 3. El total de vueltas del motor
    // 4. El estado de la prueba ( 1 = prueba concluida )
    
    texto = "{\"carga\":"+ String(lectura,2)+ ",\"velocidad\":"+ String(frecuencia)+ ",\"no_vueltas\":"+ String(vueltas_total)+ ",\"estado\":"+ String(estado)+ "}";
    texto.toCharArray(datos,80);                    // En "datos" se almacena la info en 80 caracteres
    client.publish("maquina_fatiga/datos",datos);   // Envío de información por MQTT (se especifica el tema)
    bandera = 1;                                    // bandera para "congelar" los datos a mostrar  
    while(bandera == 1){                            // permance el "congelamiento" el programa (a menos que se active la interrupción isr_reset)
    }
  }
  lectura_anterior = lectura;   // a "lectura_anterior" se le asigna el valor de la lectura actual
    
  display.fillRect(40,10,60,8,SSD1306_BLACK);   // Para mostrar en la pantalla OLED una gráfica tipo barra 
  display.setCursor(40, 10);                    // Fijar el cursor en una ubicación específica de la pantalla OLED
  sprintf(valor,"%.2f kgf", lectura);           // La cantidad fuerza de la CdC se convierte en char
  display.println(valor);                       // Fuerza de la CdC que se enviará a la pantalla OLED
  display.fillRect(40,35,60,8,SSD1306_BLACK);   // Para mostrar en la pantalla OLED una gráfica tipo barra 
  sprintf(valvueltas,"%d RPS", frecuencia);     // La velocidad en rps se convierte en char
  display.setCursor(40, 35);                    // Fijar el cursor en una ubicación específica de la pantalla OLED
  display.println(valvueltas);                  // La velocidad en rpm que se enviará a la pantalla OLED
  display.fillRect(0,55,110,8,SSD1306_BLACK);   // Para mostrar en la pantalla OLED una gráfica tipo barra 
  sprintf(valvueltas,"%d", vueltast);           // La cantidad de vueltas se convierte en char
  display.setCursor(40, 55);                    // Fijar el cursor en una ubicación específica de la pantalla OLED
  display.println(valvueltas);                  // La cantidad de vueltas que se enviará a la pantalla OLED 
  display.display();                            // Envío de los 3 mensajes al display (parece haber un error al usar 2 veces la variable valvueltas)


  // Control de tiempo para esperas no bloqueantes

  timeNow = millis();                 // a "timeNow" se le asigna el valor de la hora actual
  if (timeNow - timeLast > wait) {    // Para que se envíe un mensaje por MQTT cada 2 segundos
    timeLast = timeNow;               // Actualización de seguimiento de tiempo


    // Almacenamiento de información en formato JSON
    // 1. La fuerza sensada por la Celda de Carga
    // 2. La velocidad del motor en rpm
    // 3. El total de vueltas del motor
    // 4. El estado de la prueba ( 0 = prueba en proceso )

    texto = "{\"carga\":"+ String(lectura,2)+ ",\"velocidad\":"+ String(frecuencia)+ ",\"no_vueltas\":"+ String(vueltast)+ ",\"estado\":"+ String(estado)+"}";
    texto.toCharArray(datos,80);                    // En "datos" se almacena la info en 80 caracteres
    client.publish("maquina_fatiga/datos",datos);   // Envío de información por MQTT (se especifica el tema)
    
  }
}


// ----------------------------------------------------------------------------------- //
// --------------       FUNCIÓN P/RECIBIR MENSAJES DE LA DASHBOARD      -------------- //
// ----------------------------------------------------------------------------------- //

void callback(char* topic, byte* message, unsigned int length) {

  String messageTemp;                 // Se declara la variable String en la cual se generará el mensaje completo  
  for (int i = 0; i < length; i++) {  
    messageTemp += (char)message[i];  // Se concatena e imprime el mensaje
  }

  Serial.println (messageTemp);       // Se imprime el mensaje en el monitor serial

  
  // Accionamiento de arranque desde el Dashboard, por medio de mensajería MQTT
  // Los valores que envía la dashboard son 0 o 1
  
  if (String(topic) == "maquina/arranque") {  // Si el ESP32 recibe un mensage del tema "maquina/arranque"...
    if(messageTemp == "1"){                   // Si se recibe un "1" en messageTemp...
      Serial.println("Activa motor");         // Se imprime "Activa motor" en el monitor serial
      digitalWrite(MOTOR_INICIO, HIGH);       // En el pin 17 del ESP32 se coloca un "1" ... Arranca el motor
      messageTemp == "0";                     // Como se enclava un contacto en el contactor, se puede regresar el valor a 0
      delay(100);                             // Espera bloqueante de 100 ms
      digitalWrite(MOTOR_INICIO, LOW);        // Como se enclava un contacto en el contactor, se puede regresar el valor a LOW
    }
  }

  
  // Accionamiento de regulación de velocidad desde el Dashboard, por medio de mensajería MQTT
  // La PWM es invertida debido a la lógica invertida del optoacoplador 6N137
  // Los valores que envía la dashboard van desde 0 hasta 225
  
  if (String(topic) == "maquina/velocidad") { // Si el ESP32 recibe un mensage del tema "maquina/velocidad"...
    dutyCycle = 255 - messageTemp.toInt();    // Generación del ciclo de trabajo de la PWM invertida
    ledcWrite(ledChannel, dutyCycle);         // Coloca la PWM invertida en el GPIO 16 mediante el canal 0 
  }

}


// ----------------------------------------------------------------------------------- //
// --------------                  FUNCIÓN P/RECONECTARSE               -------------- //
// ----------------------------------------------------------------------------------- //

void reconnect() {
  while (!client.connected()) {                   // Mientras no haya conexión Wifi en el ESP32...
    Serial.print("Tratando de contectarse...");   // Imprime el mensaje en el monitor serial
    if (client.connect("ESP32Client")) {          // En caso de que la conexión SI se logre...
      Serial.println("Conectado");                // Imprime el mensaje en el monitor serial
    }
    else {                                        // En caso de que la conexión NO se logre...
      Serial.print("Conexion fallida, Error rc=");// Imprime el mensaje en el monitor serial
      Serial.print(client.state());               // Imprime el código de error en el monitor serial
      Serial.println(" Volviendo a intentar en 5 segundos");
      delay(5000);                                // Espera bloqueante de 5 segundos 
      Serial.println (client.connected ());       // Muestra estatus de conexión
    }
  }
}
