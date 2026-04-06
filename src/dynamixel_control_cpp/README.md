# Dynamixel Control - ROS 2

Control de servomotores Dynamixel AX-12 desde ROS 2 (Humble) en C++.

---

## Que necesitas

- **Ubuntu 22.04** con **ROS 2 Humble** instalado
- **Servomotores Dynamixel AX-12** (protocolo 1.0)
- **Adaptador USB** (U2D2, USB2Dynamixel o similar) conectado a `/dev/ttyUSB0`
- Paquete `dynamixel_sdk` instalado en tu workspace ROS 2

---

## Estructura del proyecto

```
src/dynamixel_control_cpp/
├── src/
│   ├── dynamixel_node.cpp   <-- Nodo hardware (habla con los motores)
│   ├── monitor_node.cpp     <-- Nodo monitor  (muestra dashboard en terminal)
│   ├── demo_node.cpp        <-- Nodo demo     (envia movimientos)
│   ├── demo_ax12.cpp        <-- Script standalone (sin ROS, directo al motor)
│   ├── monitor.cpp          <-- Monitor standalone (sin ROS)
│   ├── publisher.cpp        <-- Ejemplo basico pub/sub ROS 2
│   └── subscriber.cpp       <-- Ejemplo basico pub/sub ROS 2
├── CMakeLists.txt
├── package.xml
└── README.md                <-- Este archivo
```

---

## Conceptos clave (para novatos)

### Que es un servomotor Dynamixel?

Es un motor inteligente que puedes controlar desde tu computadora. Le dices
"ve a la posicion 500" y el motor se mueve solo hasta ahi. Ademas, te dice
en que posicion esta, cuanta fuerza esta haciendo (load) y su temperatura.

### Que es ROS 2?

Es un framework para robots. En vez de tener un solo programa gigante que
hace todo, divides tu robot en **nodos** (programas pequenos) que se comunican
entre si por **topics** (canales de mensajes).

### Por que dividir en nodos?

Imagina que tienes un robot con 12 motores. Sin ROS 2 tendrias un solo
programa enorme que hace todo: leer motores, mostrar datos, enviar comandos,
planificar movimientos... Si algo falla, todo se cae.

Con ROS 2, cada tarea es un nodo independiente:

```
┌─────────────────┐     /joint_states      ┌────────────────┐
│  dynamixel_node │ ─────────────────────> │  monitor_node  │
│   (hardware)    │   (posicion, carga,    │  (dashboard)   │
│                 │    velocidad)           └────────────────┘
│  /dev/ttyUSB0   │
│                 │ <─────────────────────  ┌────────────────┐
└─────────────────┘     /joint_commands    │   demo_node    │
                      (posiciones meta)    │ (movimientos)  │
                                            └────────────────┘
```

- Si el monitor se cae, los motores siguen funcionando.
- Si quieres cambiar como se mueven, solo modificas `demo_node`.
- Puedes agregar nodos nuevos (ej. planificador, vision) sin tocar los demas.

---

## Como compilar

Siempre en este orden:

```bash
# 1. Cargar ROS 2 (una vez por terminal)
source /opt/ros/humble/setup.bash

# 2. Ir al workspace
cd ~/enigma_ws

# 3. Compilar
colcon build --packages-select dynamixel_control_cpp

# 4. Cargar los ejecutables recien compilados
source install/setup.bash
```

> **Regla de oro:** `source ROS` -> `colcon build` -> `source install` -> ejecutar.

---

## Como ejecutar (sistema ROS 2 completo)

Necesitas **3 terminales**. En cada una, primero ejecuta:

```bash
source /opt/ros/humble/setup.bash
cd ~/enigma_ws
source install/setup.bash
```

### Terminal 1 - Nodo hardware (siempre primero)

```bash
ros2 run dynamixel_control_cpp dynamixel_node
```

Este nodo:
- Abre el puerto serie y se conecta a los motores
- Lee posicion, velocidad, carga y temperatura 20 veces por segundo
- Publica todo en el topic `/joint_states`
- Escucha comandos en `/joint_commands` y mueve los motores

### Terminal 2 - Monitor (dashboard en tiempo real)

```bash
ros2 run dynamixel_control_cpp monitor_node
```

Veras algo asi:

```
  ╔═══════════════════════════════════════════════════════════╗
  ║    MONITOR ROBOT DYNAMIXEL  (3 servos)        frame  42  ║
  ╚═══════════════════════════════════════════════════════════╝
  ── servo_9   ─────────────────────────────────────────────
    Pos:  58.9°   Vel:     0
    Load:   2.3% CW  [#-----------------------------]  baja

  ── servo_10  ─────────────────────────────────────────────
    Pos: 149.6°   Vel:     0
    Load:   1.5% CW  [------------------------------]  baja

  ── servo_12  ─────────────────────────────────────────────
    Pos: 234.0°   Vel:     0
    Load:  62.1% CW  [#############-----------------]  media
```

- Barra **verde** = carga baja (motor tranquilo)
- Barra **amarilla** = carga media (algo de resistencia)
- Barra **roja** = carga alta (mucha fuerza o bloqueo)

### Terminal 3 - Demo (enviar movimientos)

```bash
ros2 run dynamixel_control_cpp demo_node
```

Ejecuta una secuencia de movimientos en onda: cada motor se mueve a una
posicion distinta, creando un patron ondulante. Son 10 pasos, uno cada 3
segundos.

---

## Variables que se leen de cada motor

| Variable | Que mide | Para que sirve |
|----------|----------|----------------|
| **Position** | Angulo actual (0-300 grados) | Saber donde esta el motor |
| **Speed** | Velocidad de giro | Saber si se esta moviendo |
| **Load** | Fuerza/esfuerzo (0-100%) | Detectar contacto, colisiones, resistencia |
| **Temperature** | Calor interno (en Celsius) | Proteger el motor de sobrecalentamiento |

### Load: la variable mas util para robotica

Si tu robot tiene un pie y quieres saber cuando toca el suelo:

- Motor bajando libre -> Load baja (~5%)
- Pie toca el suelo -> Load **sube de golpe** (~40-80%)
- Motor bloqueado -> Load al maximo, velocidad 0

---

## Parametros configurables

Los nodos aceptan parametros por linea de comandos:

```bash
# Cambiar IDs de los servos (ej. para 12 motores)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p servo_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]"

# Cambiar puerto o baudrate
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p port:="/dev/ttyACM0" -p baudrate:=57600

# Cambiar velocidad de lectura (Hz)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p publish_rate_hz:=50.0

# Ajustar compliance (ver seccion abajo)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p compliance_margin:=4 -p compliance_slope:=64

# La demo tambien acepta IDs y numero de pasos
ros2 run dynamixel_control_cpp demo_node \
  --ros-args -p servo_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]" -p total_steps:=20
```

### Compliance: evitar que los motores tiemblen

El AX-12 usa un sistema llamado **Compliance** para mantener su posicion.
Tiene dos ajustes clave:

- **Compliance Margin**: zona muerta alrededor del goal (en unidades de
  posicion, ~0.29 grados cada una). Si el error esta dentro del margin,
  el motor no intenta corregir. Default de fabrica = 1 (muy ajustado).
- **Compliance Slope**: que tan agresivo corrige cuando el error supera el
  margin. Valores bajos = agresivo, valores altos = suave.
  Default de fabrica = 32.

Con los defaults de fabrica, los motores pueden **temblar/oscilar** porque
intentan corregir errores de 0.3 grados de forma agresiva. Esto es
especialmente notorio en motores con algo de desgaste o bajo carga.

El `dynamixel_node` configura por defecto `margin=4` y `slope=64` que dan
un comportamiento suave y estable. Puedes ajustarlos segun tu necesidad:

| Caso | margin | slope | Resultado |
|------|--------|-------|-----------|
| Motor tiembla mucho | 8 | 128 | Muy suave, menos precision |
| Operacion normal | 4 | 64 | Buen balance (default del nodo) |
| Alta precision | 2 | 32 | Mas preciso pero puede oscilar |
| Fabrica AX-12 | 1 | 32 | Puede temblar en motores usados |

```bash
# Motores viejos o con desgaste: mas suavidad
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p compliance_margin:=8 -p compliance_slope:=128

# Alta precision (solo si los motores no tiemblan)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p compliance_margin:=2 -p compliance_slope:=32
```

---

## Programas standalone (sin ROS)

Para pruebas rapidas sin levantar toda la infraestructura ROS 2:

```bash
# Demo directa: mueve 3 motores (IDs 9, 10, 12)
ros2 run dynamixel_control_cpp demo_ax12

# Monitor directo: muestra load/posicion sin ROS topics
ros2 run dynamixel_control_cpp monitor
```

Estos hablan directamente con los motores por puerto serie. Solo puede
correr uno a la vez (porque el puerto serie no se comparte).

---

## Problemas comunes

### "Error opening serial port"

Otro programa tiene el puerto abierto. Solucion:

```bash
# Ver quien usa el puerto
fuser /dev/ttyUSB0

# Matarlo
fuser -k /dev/ttyUSB0
```

> **Tip:** Si cancelas un programa, usa **Ctrl+C** (lo mata). Nunca **Ctrl+Z**
> (lo pausa pero deja el puerto bloqueado).

### El motor tiembla/oscila

El motor intenta corregir su posicion de forma muy agresiva. Ver la
seccion **Compliance** mas arriba. Solucion rapida: subir margin y slope.

### El motor no se mueve (Error 0x02 = Angle Limit)

El motor esta en "modo rueda". Los nodos lo detectan y corrigen
automaticamente. Si usas el script standalone, tambien lo maneja.

### Error 0x04 = Overheating

El motor esta caliente. Dejalo enfriar unos minutos. Si persiste, revisa
la ventilacion o reduce el torque.

### No se cual puerto usar

```bash
# Ver puertos disponibles
ls /dev/ttyUSB* /dev/ttyACM*

# Identificar: desconecta el USB, mira la lista, reconecta y mira que aparecio nuevo
```

### Permisos del puerto

```bash
# Solucion permanente (requiere cerrar sesion y volver a entrar)
sudo usermod -aG dialout $USER

# Solucion rapida
sudo chmod 666 /dev/ttyUSB0
```

---

## Siguiente paso: tu propio nodo

Para agregar comportamiento nuevo (ej. caminar, seguir un objeto), crea un
nodo que publique en `/joint_commands`. No necesitas tocar el hardware node:

```cpp
// mi_controlador.cpp - esqueleto basico
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MiControlador : public rclcpp::Node {
public:
    MiControlador() : Node("mi_controlador") {
        // Leer estado de los motores
        state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                // Aqui puedes leer msg->position, msg->effort (load), etc.
            });

        // Enviar comandos
        cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", 10);
    }
};
```

Agrega el ejecutable al `CMakeLists.txt`, compila, y ya tienes un nodo
nuevo que controla tu robot sin modificar nada del codigo existente.
