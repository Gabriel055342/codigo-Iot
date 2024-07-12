#include <WiFi.h>
#include <PubSubClient.h>

// Configuración de conexión WiFi
const char* nombre_red = "SISTELCEL_JIMMY";
const char* contraseña_red = "mch0106386071jj***";

// Configuración de MQTT
const char* mqtt_token = "mnyuOKOt6cHL642tmspe";
const char* thingsboardServer = "192.168.0.118";

WiFiClient espClient;
PubSubClient client(espClient);

// Variables
float distancia = 0;
float distancia_medida = 0;
int pin_Echo = 19; // Pin Echo conectado a GPIO14 (D15) en ESP32
int pin_Trig = 17; // Pin Trig conectado a GPIO13 (D13) en ESP32
int pin_motor = 12; // Pin Motor conectado a GPIO12 (D12) en ESP32
int pin_led_verde = 32;
int pin_led_naranja = 33;
int pin_led_rojo = 25;

void Conectar_wifi(const char* nombre, const char* contraseña) {
    // Iniciar conexión WiFi
    WiFi.begin(nombre, contraseña);
    Serial.print("Conectando a la red ");
    Serial.print(nombre);

    int intentos_max = 20;
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < intentos_max) {
        delay(500);
        Serial.print(".");
        intentos++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi conectado");
        Serial.print("Dirección IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("");
        Serial.println("Error: No se pudo conectar a la red WiFi");
    }
}

// Función de reconexión MQTT
void reconnect() {
    while (!client.connected()) {
        Serial.print("Intentando conexión MQTT...");
        if (client.connect("ESP32Client", mqtt_token, NULL)) {
            Serial.println("conectado");
            // Suscribirse al topic de comandos para recibir comandos de ThingsBoard
            client.subscribe("v1/devices/me/rpc/request/+");
        } else {
            Serial.print("falló, rc=");
            Serial.print(client.state());
            Serial.println(" intentamos de nuevo en 5 segundos");
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    // Aquí puedes manejar los mensajes recibidos del servidor MQTT
    Serial.print("Mensaje recibido [");
    Serial.print(topic);
    Serial.print("]: ");
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);

    if (message.indexOf("params") > 0) {
        if (message.indexOf("true") > 0) {
            digitalWrite(pin_motor, HIGH); // Encender el motor
            Serial.println("Ventilador encendido");
        } else if (message.indexOf("false") > 0) {
            digitalWrite(pin_motor, LOW); // Apagar el motor
            Serial.println("Ventilador apagado");
        }
    }
}

long Medir_Distancia_Tanque(int trigger, int echo) {
    // Inicializar el pin para medir 
    pinMode(trigger, OUTPUT);
    // Apagar el pin para medir 
    digitalWrite(trigger, LOW);
    // Pequeña espera 
    delayMicroseconds(2);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    pinMode(echo, INPUT);
    
    // Medir duración del pulso de eco
    long duracion = pulseIn(echo, HIGH, 30000); // 30000 microsegundos es el tiempo máximo de espera

    return duracion;
}

void setup() {
    // Inicializar puerto serial
    Serial.begin(115200);

    // Conectar a WiFi
    Conectar_wifi(nombre_red, contraseña_red);

    // Configurar cliente MQTT
    client.setServer(thingsboardServer, 1883);
    client.setCallback(callback);

    // Inicializar pines
    pinMode(pin_motor, OUTPUT);
    pinMode(pin_Trig, OUTPUT);
    pinMode(pin_Echo, INPUT);
    pinMode(pin_led_verde, OUTPUT);
    pinMode(pin_led_naranja, OUTPUT);
    pinMode(pin_led_rojo, OUTPUT);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Medir distancia
    distancia_medida = 0.01723 * Medir_Distancia_Tanque(pin_Trig, pin_Echo);
   
    Serial.print("Valor medido: ");
    Serial.println(distancia_medida);
    

    Serial.print("VALOR CALCULADO.... ");
    Serial.println((21.68-distancia_medida) * 100 / 19);
    distancia = (21.68-distancia_medida)*100/19;



    if (distancia <50)
    {
      digitalWrite(pin_led_verde, HIGH);
      digitalWrite(pin_led_rojo, LOW);
   
      digitalWrite(pin_led_naranja, LOW);
      digitalWrite(pin_motor, LOW); // apagar el motor

    }else if (50 <= distancia < 85)
    {
      digitalWrite(pin_led_naranja, HIGH);
      digitalWrite(pin_led_rojo, LOW);
      digitalWrite(pin_led_verde, LOW);
      digitalWrite(pin_motor, LOW); // apagar el motor
      

    }else
    {
      digitalWrite(pin_led_rojo, HIGH);
      digitalWrite(pin_led_verde, LOW);
      digitalWrite(pin_led_naranja, LOW);
      digitalWrite(pin_motor, LOW); // apagar el motor
    }
    
    if (distancia >= 100)
    {

      digitalWrite(pin_motor, HIGH); // Encender el motor
    }

    // Publicar datos en ThingsBoard
    String payload = "{\"Nivel\":" + String(distancia) + "}";
    client.publish("v1/devices/me/telemetry", payload.c_str());

    delay(500); // Esperar medio segundo antes de la próxima lectura
}

