---
# VNT_AutodrivePlanner
---

# Mapeo y Planificación Global de Trayectorias en AutoDRIVE — LPA*

Proyecto para el simulador **AutoDRIVE (F1TENTH)**, incluye mapeo del entorno mediante SLAM Toolbox (Parte A) y planificación global de trayectoria con **LPA\* (Lifelong Planning A\*)** + suavizado con B-Spline, integrado en ROS 2 (Parte B).

---

## 📁 Estructura del proyecto

```
.
├── README.md
├── requirements.txt
├── .gitignore
├── videos/
└── src/
    └── f1tenth_global_planner/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── f1tenth_global_planner
        ├── f1tenth_global_planner/
        │   ├── __init__.py
        │   ├── planner_node.py         # nodo ROS 2: LPA* sobre el mapa real
        │   ├── smoothing_node.py       # nodo ROS 2: suavizado B-Spline
        │   ├── grid_from_occupancy.py  # adaptador OccupancyGrid -> Grid
        │   ├── smoothing.py            # funciones de ajuste/evaluación de spline
        │   └── python_motion_planning/ 
        ├── launch/
        │   └── planning.launch.py      # map_server + lifecycle_manager + planner + smoother + rviz
        ├── config/
        │   └── planning.rviz           # configuración de RViz2 con los displays ya armados
        └── maps/
            ├── F1tenth_Map.pgm         # mapa generado en Parte A
            └── F1tenth_Map.yaml
```

---

## ✅ Requisitos

| Componente | Versión / Detalle |
|---|---|
| Sistema operativo | Ubuntu 22.04 |
| ROS 2 | Humble |
| Python | 3.10 (venv recomendado) |
| AutoDRIVE Simulator | Linux, escenario F1TENTH |
| Dependencias Python | ver [`requirements.txt`](./requirements.txt) (`numpy`, `scipy`, `matplotlib`, `cvxopt`, `tqdm`, `osqp`) |

---

## 📥 Clonación e instalación

```bash
mkdir -p ~/autodrive_ws/src
cd ~/autodrive_ws/src
git clone https://github.com/Gennalop/VNT_AutodrivePlanner f1tenth_project
```

> Este repositorio ya trae la estructura `src/f1tenth_global_planner/`, así que si prefieres clonarlo directamente como la carpeta `src` de tu workspace, ajusta el paso anterior según te convenga — solo asegúrate de que `src/f1tenth_global_planner` termine dentro de `~/autodrive_ws/src/`.

Crear y activar el entorno virtual, e instalar dependencias:

```bash
cd ~/autodrive_ws
python3 -m venv venv
source venv/bin/activate
pip install -r src/f1tenth_project/requirements.txt
```

---

## 🔨 Compilación

```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
colcon build --packages-select f1tenth_global_planner --symlink-install
source install/setup.bash
```

---

## ▶️ Ejecución

### Parte A — Mapeo (SLAM Toolbox)

> Resumen de los pasos; para el detalle completo con explicación de cada parámetro, ver la documentación original ....

1. **Terminal 1 — Simulador:**
   ```bash
   cd ~/Downloads/AutoDRIVE_Sim
   ./"AutoDRIVE Simulator.x86_64"
   ```
   Selecciona el escenario F1TENTH y dale Play.

2. **Terminal 2 — Bridge AutoDRIVE ↔ ROS 2:**
   ```bash
   cd ~/autodrive_ws
   source /opt/ros/humble/setup.bash && source venv/bin/activate && source install/setup.bash
   ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
   ```
   Espera 3–5 segundos a que `/clock`, `/tf` y el LiDAR empiecen a publicar.

3. **Terminal 3 — Teleoperación:**
   ```bash
   ros2 run autodrive_f1tenth teleop_keyboard
   ```

4. **Terminal 4 — SLAM Toolbox:**
   ```bash
   CFG=$(realpath ~/autodrive_ws/src/f1tenth_project/slam_config/mapper_params_online_async.yaml)
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$CFG use_sim_time:=true
   ```

5. Mueve el vehículo lentamente por todo el circuito hasta cerrar el mapa.

6. Guarda el mapa desde el panel `SlamToolboxPlugin` en RViz (botón **Save Map**), apuntando a:
   ```
   ~/autodrive_ws/src/f1tenth_project/src/f1tenth_global_planner/maps/F1tenth_Map
   ```
   Esto genera `F1tenth_Map.pgm` y `F1tenth_Map.yaml`.

### Parte B — Planificación global (LPA\*) + suavizado

Con el simulador y el bridge de la Parte A **ya corriendo** (pasos 1 y 2 de arriba):

1. **Terminal nueva — Pipeline de planificación:**
   ```bash
   cd ~/autodrive_ws
   source /opt/ros/humble/setup.bash && source venv/bin/activate && source install/setup.bash
   ros2 launch f1tenth_global_planner planning.launch.py
   ```
   Esto levanta: `map_server` (publica `/map` desde `F1tenth_Map.yaml`), `lifecycle_manager`, `planner_node` (LPA\*), `smoothing_node` (B-Spline) y RViz2 con la configuración guardada en `config/planning.rviz`.

2. En RViz, usa la herramienta **"2D Goal Pose"** y haz clic en cualquier punto libre del mapa.

3. Verás en tiempo real:
   - `/expand_markers` — nodos explorados por LPA\* mientras busca la ruta.
   - `/raw_path` — ruta cruda encontrada por LPA\*.
   - `/smoothed_path` — ruta suavizada con B-Spline.
---

## 🎥 Videos y evidencias


---

## 🧠 Descripción de los algoritmos, variables importantes y modificaciones realizadas

### Algoritmo de planificación: LPA\* (Lifelong Planning A\*)

LPA\* es una variante incremental de A\* pensada para entornos que cambian dinámicamente: en vez de recalcular la ruta completa desde cero ante un cambio en el mapa, reutiliza información de búsquedas previas (valores `g` y `rhs` por nodo) para actualizar solo la porción afectada. En este proyecto se usa la implementación de la librería [`python_motion_planning`](https://github.com/ai-winter/python_motion_planning) (vendorizada dentro de `f1tenth_global_planner/f1tenth_global_planner/python_motion_planning/`).

### Suavizado: B-Spline

La ruta cruda que entrega LPA\* (secuencia de celdas discretas) se convierte a coordenadas del mundo y se ajusta con una spline suave mediante `scipy.interpolate` (`smoothing.py`), controlada por una tolerancia de desviación máxima respecto a los waypoints originales.

### Variables importantes

| Variable | Ubicación | Descripción |
|---|---|---|
| `occ_threshold` | `grid_from_occupancy.py` | Umbral de probabilidad (0–100) a partir del cual una celda del `OccupancyGrid` se considera obstáculo. |
| `SMOOTHING_TOLERANCE_M` | `smoothing_node.py` | Tolerancia máxima (en metros) que la spline puede desviarse de los waypoints crudos. |
| Frame de lookup de TF | `planner_node.py` → `get_start_world()` | `map → f1tenth_1`; define de dónde se toma la pose de inicio real del vehículo. |
