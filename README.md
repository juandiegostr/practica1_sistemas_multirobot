# ROS 2 Multi-Robot Workspace

Este repositorio contiene un entorno de trabajo para la simulación de robots móviles utilizando ROS 2 y Gazebo Sim. El proyecto está estructurado para facilitar la integración de la descripción del robot y su lanzamiento en simulación.

## 📦 Paquetes Principales

Actualmetne el workspace cuenta con dos paquetes clave en `src/`:

### 1. `practica1`
Este paquete se encarga de la descripción física y visual del robot.
- **Descripción del Robot**: Contiene los archivos Xacro/URDF que definen la estructura, cinemática y propiedades físicas del robot.
- **Archivos Clave**:
  - `urdf/robotito.urdf.xacro`: Archivo principal que ensambla el modelo del robot.
  - `urdf/gazebo_control.xacro`: Plugins y configuraciones específicas para el control en Gazebo.
  - `urdf/inertial_macros.xacro`: Macros reutilizables para propiedades inerciales.

### 2. `robot_bringup`
Este paquete orquesta el lanzamiento de la simulación y la integración con ROS 2.
- **Launch Files**: Scripts de Python para iniciar nodos y simuladores.
- **Configuración**: Parámetros para el puente entre ROS 2 y Gazebo.
- **Mundos**: Archivos de entorno para la simulación.
- **Archivos Clave**:
  - `launch/robot_bringup.launch.py`: Launch file principal.
  - `config/bridge_config.yaml`: Configuración del `ros_gz_bridge`.
  - `world/my_world.sdf`: Mundo de simulación en Gazebo.

## Cómo Construir el Workspace

Asegúrate de tener instaladas las dependencias de ROS 2 y Gazebo Sim.

1. Navega a la raíz del workspace:
   ```bash
   cd /ros2_ws_multirobots
   ```

2. Construye los paquetes usando `colcon`:
   ```bash
   colcon build --symlink-install
   ```

3. Fuentea el entorno:
   ```bash
   source install/setup.bash
   ```

## Cómo Ejecutar la Simulación

Para lanzar la simulación completa (Gazebo + Robot State Publisher + Bridge), ejecuta:

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

### ¿Qué hace este comando?
1. **Procesa el URDF**: Convierte `robotito.urdf.xacro` a XML puro.
2. **Robot State Publisher**: Publica la descripción del robot y transformaciones estáticas en `/robot_description` y `/tf_static`.
3. **Gazebo Sim**: Inicia el simulador con el mundo `my_world.sdf`.
4. **Spawn Entity**: Spawnea (genera) el robot `robotito` en la simulación.
5. **ROS GZ Bridge**: Inicia el puente bidireccional entre ROS 2 y Gazebo para los Topics configurados.

## lendo Bridge Configuration (`ros_gz_bridge`)

El puente (`bridge_config.yaml`) está configurado para comunicar los siguientes Topics:

| Topic ROS 2 | Topic Gazebo | Dirección | Tipo de Mensaje |
|--------------|---------------|-----------|-----------------|
| `/clock`     | `/clock`      | GZ -> ROS | `rosgraph_msgs/msg/Clock` |
| `/joint_states` | `/joint_states` | GZ -> ROS | `sensor_msgs/msg/JointState` |
| `/tf`        | `/tf`         | GZ -> ROS | `tf2_msgs/msg/TFMessage` |
| `/cmd_vel`   | `/cmd_vel`    | ROS -> GZ | `geometry_msgs/msg/Twist` |

Esto permite controlar el robot publicando en `/cmd_vel` y recibir el estado de las articulaciones y transformaciones en ROS 2.
