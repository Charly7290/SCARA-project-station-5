# SCARA-project-station-5
### **Comandos de ejecución dentro de SCARA_Project/SCARA-project-station-5:**
**Terminal 1:** \
curl -fsSL https://get.docker.com | sudo sh #Descargar Docker\
./tools/microros_bringup/run_agent.sh #Ejecutar Agente MicroROS\
**Terminal 2:** \
source /opt/ros/jazzy/setup.bash \
ros2 topic pub /scara_cmd std_msgs/msg/String "data: 'STOP'" --once #Detener SCARA\
ros2 topic pub /scara_cmd std_msgs/msg/String "data: 'HOME'" --once #Enviar SCARA a estado Home\
ros2 topic pub /scara_cmd std_msgs/msg/String "data: 'MANUAL'" --once #Enviar SCARA a estado Manual\
ros2 topic pub /scara_cmd std_msgs/msg/String "data: 'GO TO: 15, 15, 0, ON'" --once #coordenadas de acuerdo al W-Space del SCARA y la activación de la herramienta, es decir, GO TO: x, y, z, ON/OFF\
ros2 topic pub /scara_cmd std_msgs/msg/String "data: 'PICK&PLACE'" --once #El SCARA hará su respectiva tarea

### **Comandos de ejecución para el sistema IoT:**
En computador: Raspberry Pi Pico --> agente --> /scara_telemetry topic --> bridge.js --> Mosquitto --> server.js --> MariaDB --> Grafana 

**INSTALACIONES PARA IOT:**\
**Mosquitto:**\
sudo apt update\
sudo apt install -y mosquitto mosquitto-clients\
sudo mosquitto_passwd -c /etc/mosquitto/passwd scara   # pide la contraseña \
sudo tee /etc/mosquitto/conf.d/scara.conf <<'EOF'\
listener 1883\
allow_anonymous false\
password_file /etc/mosquitto/passwd\
EOF\
sudo chown mosquitto:mosquitto /etc/mosquitto/passwd\
sudo chmod 640 /etc/mosquitto/passwd\
sudo systemctl enable mosquitto\
sudo systemctl restart mosquitto\
systemctl is-active mosquitto    # debe imprimir: active

**Node.js:**\
sudo apt install -y curl\
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -\
sudo apt install -y nodejs\
node -v    # 20.20.2 o superior

**MariaDB, Apache, PHP y phpMyAdmin:**\

sudo apt install -y apache2 mariadb-server php libapache2-mod-php php-mysql\
sudo systemctl enable --now apache2 mariadb\
sudo mysql_secure_installation\
sudo apt install -y phpmyadmin php-mbstring\

- `mysql_secure_installation`: en "current password" pulse Enter; responda `n` a
  unix_socket y a cambiar la contraseña de root, y `Y` a todo lo demás.
- phpMyAdmin: servidor web `apache2`; configurar la base con dbconfig-common: `yes`;
  deje vacías las contraseñas.

**Base de datos y usuario:**\

Ejecute `sudo mariadb` y escriba (reemplace `<DB_PASS>`): \

*SQL*\
CREATE DATABASE IF NOT EXISTS scara;\
CREATE USER IF NOT EXISTS 'scara'@'%' IDENTIFIED BY '<DB_PASS>';\
GRANT ALL PRIVILEGES ON scara.* TO 'scara'@'%';\
FLUSH PRIVILEGES;\
USE scara;\
CREATE TABLE `values` (
  ID INT AUTO_INCREMENT PRIMARY KEY,\
  numeric_value DOUBLE NOT NULL,\
  source_id INT NOT NULL,\
  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP\
);\
EXIT;

**Grafana**\
sudo apt install -y apt-transport-https wget gpg\
sudo mkdir -p /etc/apt/keyrings\
wget -q -O - https://apt.grafana.com/gpg.key | gpg --dearmor | sudo tee /etc/apt/keyrings/grafana.gpg > /dev/null\
echo "deb [signed-by=/etc/apt/keyrings/grafana.gpg] https://apt.grafana.com stable main" | sudo tee /etc/apt/sources.list.d/grafana.list\
sudo apt update\
sudo apt install -y grafana\
sudo systemctl enable --now grafana-server\

Abra http://localhost:3000 (admin / admin la primera vez). Agregue un origen de datos\
MySQL: host `127.0.0.1:3306`, base `scara`, usuario `scara`.

**Entorno de ROS2**\

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc\

**Dependencias de Node y contraseñas**\

Desde la raíz del repositorio (ROS 2 debe estar cargado antes de instalar `scara-bridge`):\

source /opt/ros/jazzy/setup.bash\
cd iot && npm install && cp .env.example .env && cd ..\
cd scara-bridge && npm install && cp .env.example .env && cd ..\

Edite los dos archivos `.env` con las contraseñas reales. Estos archivos no se suben a git.\

**Verificación**

- `systemctl is-active mosquitto apache2 mariadb grafana-server` imprime `active` cuatro veces.
- http://localhost/phpmyadmin permite entrar con el usuario `scara`.
- `mariadb -u scara -p -h 127.0.0.1` abre la consola de MariaDB.

**Terminal 3: escritor de la base de datos** \
cd iot && node --env-file=.env server.js 

**Terminal 4: puente ROS 2 a MQTT** \
source /opt/ros/jazzy/setup.bash \
cd scara-bridge && node --env-file=.env bridge.js 

**Terminal 5 (opcional): ver los mensajes MQTT**
mosquitto_sub -h localhost -u scara -P <MQTT_PASS> -t 'scara/#' -v 



# **Descripción de los módulos de software**

La arquitectura de software está estructurada en dos niveles: ***software de alto nivel y software de bajo nivel***. El nivel superior se encarga de la lógica de operación, la comunicación y la generación de las órdenes para el manipulador SCARA, mientras que el nivel inferior se encarga de ejecutar dichas órdenes sobre los actuadores y realizar la lectura de los sensores.

---

## **1. Arquitectura de software de alto nivel**

El software de alto nivel está basado en ROS2 y se comunica con el sistema de bajo nivel mediante un agente micro-ROS, utilizando la conexión USB/CLI. En este nivel se encuentran los módulos encargados de coordinar el funcionamiento general de la estación.

<aside>

### **Nodo de comunicación con la estación anterior**

Este módulo se encarga de establecer la comunicación entre la Estación 5 (la nuestra) y la estación anterior. Su función principal es recibir la señal que indica que el material necesario para iniciar el proceso se encuentra disponible.

La estación permanece en el estado *IDLE* mientras espera esta comunicación. Una vez recibida la señal correspondiente, se permite el inicio del ciclo de trabajo y la máquina de estados puede avanzar hacia el estado *PICK_CASE*.

</aside>

<aside>

### **Nodo Pick&Place**

Este módulo coordina las operaciones de manipulación necesarias para realizar el ensamblaje. Se divide principalmente en las etapas de *PICK_CASE, PLACE_CASE, PICK_PCB* e *INSERT_PCB*.

Para cada operación se determina una posición objetivo mediante el cálculo de la cinemática inversa y posteriormente se ejecutan los movimientos necesarios del manipulador.

Durante cada operación se supervisan los finales de carrera ubicados en la base del manipulador. Si alguno se activa durante el movimiento, la operación se interrumpe y el sistema pasa al estado HOMING.

</aside>

<aside>

### **Nodo de modificación de posición**

Este módulo se encarga de establecer las posiciones objetivo requeridas por el manipulador para realizar las diferentes operaciones del proceso.

Las posiciones corresponden principalmente a los puntos de recogida, ensamblaje, inserción y posición de origen del manipulador. Estas posiciones son utilizadas posteriormente por el sistema de control para generar los movimientos correspondientes.

</aside>

<aside>

### **Nodo de modificación de velocidad**

Este módulo permite establecer las velocidades de movimiento del manipulador. Las velocidades pueden ser modificadas de acuerdo con la operación que se esté ejecutando, permitiendo controlar el desplazamiento de los diferentes elementos del sistema durante las etapas del proceso.

</aside>

<aside>

### **Nodo Homing**

El nodo de Homing se encarga de llevar el manipulador a su posición de referencia u origen.

Durante este procedimiento se desactiva inicialmente la ventosa y se calcula la cinemática inversa correspondiente a la posición de origen. Posteriormente se ejecutan los movimientos de los motores de los links 1 y 2 y del servomotor del link 3.

Durante el movimiento se realizan lecturas de los encoders y finales de carrera para verificar la posición del manipulador. Si los motores todavía no se encuentran en la posición de origen, el movimiento continúa. Cuando se alcanza la posición correspondiente, el sistema pasa al estado *IDLE*. Además, este estado puede ser activado como mecanismo de recuperación cuando se detecta la activación de uno de los finales de carrera de seguridad ubicados en la base del manipulador.

De esta manera, el procedimiento de Homing cumple una doble función: establecer la posición de referencia del manipulador durante la inicialización y finalización del ciclo, y recuperar una posición segura cuando se detecta una condición de límite durante las operaciones de Pick&Place.

</aside>

<aside>

### **Nodo de comunicación con la estación siguiente**

Este módulo se encarga de informar a la estación siguiente que la operación de la Estación 5 ha finalizado.

Una vez completado el procesamiento de las cuatro unidades y realizado el procedimiento de Homing, se genera la señal de finalización que permite a la estación siguiente continuar con su proceso.

</aside>

---

## 2. Integración de MoveIt 2

Para la planificación y ejecución de los movimientos del manipulador se contempla el uso de **MoveIt 2**, framework integrado en ROS 2 especializado en la planificación de movimiento de robots manipuladores.

MoveIt 2 estará asociado al módulo de ***Pick&Place***, encargado de ejecutar las diferentes operaciones de la estación. A partir de una posición objetivo, como la ubicación del CASE, PCB o posición de ensamblaje, MoveIt 2 permitirá calcular la configuración de las articulaciones mediante cinemática inversa y planificar la trayectoria necesaria para que el manipulador alcance dicha posición. De esta manera, el nodo Pick&Place definirá la operación que debe realizarse y la posición objetivo, mientras que MoveIt 2 se encargará de apoyar la planificación del movimiento necesario para alcanzar dicha posición.

### Ubicación dentro de la arquitectura

> Aunque MoveIt 2 no aparece como un nodo independiente en el diagrama de arquitectura de alto nivel, se considera una herramienta utilizada por el módulo Pick&Place para la planificación del movimiento. Por esta razón, su función se encuentra conceptualmente dentro de dicho módulo y no representa un estado independiente de la máquina de estados.
> 

---

## **3. Agente micro-ROS**

El ***agente micro-ROS*** funciona como intermediario entre ROS2 y el firmware que se ejecuta en el sistema de control de bajo nivel. Los módulos de alto nivel generan las órdenes correspondientes y el agente permite transmitirlas hacia el firmware mediante la comunicación *USB/CLI.* De manera similar, la información obtenida desde sensores y actuadores puede ser utilizada por el sistema de alto nivel.

---

## **4. Arquitectura de software de bajo nivel**

El software de bajo nivel está constituido principalmente por el firmware, encargado de realizar la interacción directa con los elementos físicos del manipulado.

<aside>

### **Movimiento de los brazos**

Este módulo recibe las posiciones y trayectorias generadas en el nivel superior y las convierte en acciones de control para los motores del manipulador. Para ello utiliza las configuraciones de velocidad y los pines PWM correspondientes a cada actuador. Dentro de este control se encuentran los posicionamientos correspondientes al CASE y a la PCB.

</aside>

<aside>

### **Sensado**

Se encarga de obtener la información proveniente de los sensores del sistema.

Incluye principalmente:

- Lectura de encoders.
- Lectura de finales de carrera.

Esta información permite verificar si los motores y el manipulador han alcanzado las posiciones requeridas.

</aside>

<aside>

### **Movimiento del efector final**

Controla el movimiento del tercer eje del manipulador mediante el servomotor. También permite realizar el posicionamiento vertical necesario durante las operaciones de recogida, ensamblaje e inserción.

Además, controla las acciones necesarias para manipular las piezas como la activación y desactivación de la ventosa, utilizada para recoger y transportar los elementos durante el proceso.

</aside>

---

## 5. Gestión de errores y condición de seguridad

Como parte de la estrategia de control de la Estación, se implementa una condición de seguridad mediante finales de carrera ubicados en la base del manipulador.

Estos sensores tienen como objetivo detectar una situación en la que alguno de los brazos se aproxima a una zona límite y podría entrar en contacto con la base del robot.

La condición se supervisa durante las diferentes operaciones de movimiento. En cada una de estas operaciones se realiza el proceso de lectura de sensores; por lo tanto, la activación de un final de carrera no se considera una transición normal de la máquina de estados, sino una condición de error/seguridad que tiene prioridad sobre la operación actual.

Una vez detectada esta condición, el sistema abandona el movimiento que estaba ejecutando y procede a realizar el Homing. Después de recuperar la posición de referencia, el sistema puede retornar a un estado seguro, de acuerdo con la lógica definida para la recuperación.

Esto permite evitar que el manipulador continúe desplazándose hacia una posición que pueda generar un choque con su propia estructura.

---

## **6. Protocolo de comunicación con las estaciones adyacentes**

La estación se encuentra integrada dentro de una línea de producción, por lo que debe mantener comunicación tanto con la estación anterior como con la estación siguiente.

<aside>

### **Inicio del proceso**

Al iniciar el sistema, la Estación ejecuta los estados *INIT* y *HOMING,* configurando sus componentes y llevando el manipulador a la posición de referencia.

Una vez finalizado el Homing, la estación pasa a *IDLE*.

En este estado permanece esperando la comunicación proveniente de la estación anterior. Cuando recibe la señal que indica que el material necesario está disponible, el nodo de comunicación informa a la máquina de estados y se inicia el ciclo de producción.

</aside>

<aside>

### **Final del proceso**

Después de realizar una operación de inserción, la máquina de estados verifica la cantidad de unidades procesadas.

Si la cantidad es menor a cuatro, el proceso vuelve a *PICK_CASE* para iniciar el procesamiento de la siguiente unidad.

Cuando la cantidad alcanza cuatro unidades, el sistema realiza nuevamente el procedimiento de *HOMING*.

Una vez finalizado el Homing, la Estación envía la señal correspondiente a la estación siguiente para indicar que el proceso ha terminado.

</aside>
