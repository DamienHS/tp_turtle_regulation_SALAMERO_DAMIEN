# TP Turtle Regulation — SALAMERO DAMIEN

Ce dépôt contient un TP de régulation pour la tortue de **turtlesim** sous ROS 2 (Jazzy Jalisco, Ubuntu 24.04).  
Trois parties sont implémentées :

- **Partie 1 : Régulation en cap**  
  - Publication d’un waypoint  
  - Nœud `turtle_regulator` pour faire tourner la tortue vers le waypoint  
- **Partie 2 : Régulation en distance**  
  - Calcul de la distance euclidienne à la cible  
  - Commande linéaire proportionnelle à l’erreur  
  - Topic `is_moving` indiquant si la tortue est en mouvement  
- **Partie 3 : Service SetWayPoint**  
  - Interface ROS 2 `turtle_interfaces/srv/SetWayPoint`  
  - Service `/set_waypoint_service` pour modifier dynamiquement le waypoint  

---

## Prérequis

- Ubuntu 24.04  
- ROS 2 Jazzy Jalisco  
- `colcon` et Python3  

---

## Installation & Build

```bash
# Cloner le dépôt dans votre workspace ROS 2
cd ~/ros2_ws/src
git clone https://github.com/.../tp_turtle_regulation_SALAMERO_DAMIEN.git

# Retour à la racine du workspace
cd ~/ros2_ws

# Nettoyer anciens builds
rm -rf build/ install/ log/

# Compiler tous les packages
colcon build

# Source l’environnement (toujours après build)
source install/setup.bash

```

---

## Usage

### 1. Lancer le simulateur

```bash
ros2 run turtlesim turtlesim_node
```

### 2. Partie 1 — Régulation en cap

- **Publisher**  
  ```bash
  ros2 run turtle_regulation set_way_point
  ```
  Publie à 1 Hz sur `way_point` les coordonnées par défaut `(7.0, 7.0)`.

- **Régulateur en cap**  
  ```bash
  ros2 run turtle_regulation turtle_regulator \
    --ros-args -p Kp:=<valeur>
  ```
  | Kp   | Comportement                         |
  |:----:|:-------------------------------------|
  | 0.5  | Rotation lente, pas d’oscillation   |
  | 1.0  | Bon compromis, rapide sans dépasse   |
  | 2.0  | Très rapide, oscillations légères    |

### 3. Partie 2 — Régulation en distance

```bash
ros2 run turtle_regulation turtle_regulator \
  --ros-args -p Kp:=1.0 -p Kp1:=0.5 -p distance_tolerance:=0.1
```

- `Kp1` pour la vitesse linéaire  
- `distance_tolerance` pour arrêter la commande  
- Topic `/is_moving` (std_msgs/Bool) :
  - `True` si l’erreur linéaire > tolérance  
  - `False` sinon  

### 4. Partie 3 — Service SetWayPoint

- **Définition**  
  `turtle_interfaces/srv/SetWayPoint.srv` :
  ```srv
  float32 x
  float32 y
  ---
  bool res
  ```
- **Serveur** sur `/set_waypoint_service`  
- **Appel du service** :
  ```bash
  ros2 service call /set_waypoint_service \
    turtle_interfaces/srv/SetWayPoint "{x: 2.5, y: 8.0}"
  ```

---

## Structure du dépôt

```
tp_turtle_regulation_SALAMERO_DAMIEN/
├── turtle_regulation/
│   ├── setup.cfg
│   ├── setup.py
│   └── turtle_regulation/
│       ├── set_way_point.py
│       └── turtle_regulator.py
└── turtle_interfaces/
    ├── CMakeLists.txt
    ├── package.xml
    └── srv/
        └── SetWayPoint.srv
```

---

## Git

Les trois commits clés :

1. **Partie 1** – publication waypoint & régulation en cap  
2. **Partie 2** – régulation en distance & topic `is_moving`  
3. **Partie 3** – création du package `turtle_interfaces` et service SetWayPoint  
```
