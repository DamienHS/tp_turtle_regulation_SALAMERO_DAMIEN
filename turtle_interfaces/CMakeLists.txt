cmake_minimum_required(VERSION 3.5)
project(turtle_interfaces)

# ament & génération d’interfaces
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Génère les .idl/.h/.py à partir du srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWayPoint.srv"
)

# Permet d’exporter rosidl_default_runtime en dépendance d’exécution
ament_export_dependencies(rosidl_default_runtime)

# Termine la configuration du package
ament_package()

