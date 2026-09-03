Nodo ROS 2 que implementa un controlador **LQR** para seguimiento de trayectoria en el simulador **AutoDRIVE** (F1TENTH), como parte del Proyecto Parte 2: *Seguimiento de Trayectorias y Control*.
El nodo lee la trayectoria global suavizada (`/smoothed_path`, generada en la Parte 1), calcula el error lateral y de orientación respecto a esa trayectoria, y publica comandos de dirección y aceleración para que el vehículo la siga de forma autónoma. También cuenta vueltas completadas y mide el tiempo de cada una.

## Tópicos
 
**Se suscribe a:**
 
| Tópico | Tipo | Uso |
|---|---|---|
| `/smoothed_path` | `nav_msgs/Path` | Trayectoria global a seguir (Parte 1) |
| `/autodrive/f1tenth_1/left_encoder` | `sensor_msgs/JointState` | Encoder rueda izquierda (se usa `position`, ver nota abajo) |
| `/autodrive/f1tenth_1/right_encoder` | `sensor_msgs/JointState` | Encoder rueda derecha |
| TF `map -> f1tenth_1` | — | Pose del vehículo |
 
**Publica en:**
 
| Tópico | Tipo | Rango |
|---|---|---|
| `/autodrive/f1tenth_1/steering_command` | `std_msgs/Float32` | radianes, límite ±0.5236 rad (±30°) |
| `/autodrive/f1tenth_1/throttle_command` | `std_msgs/Float32` | normalizado, ±1.0 |
