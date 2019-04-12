# catkin_rovert_ws
Repositorio de código para el Rover que contiene todos los packages y configuraciones de ROS en formato de Catkin Workspace para facilitar control de versiones y cambios.

## Requerimientos:
Este desarrollo está basado en la distro Kinetic de ROS en Ubuntu 16.04LTS

Se deben instalar los siguientes packages de ROS para poder correr todos los nodos lanzados por los launchfiles

-ros-kinetic-navigation 
-explore_lite
-slam-gmapping
-rosserial
-rosserial_arduino


## Configuracion inicial
Para usar este workspace se debe poner en la carpeta home del sistema operativo y realizar cambios al archivo MakeFile que se encuentra en /build . Se debe buscar y reemplazar todas las apariciones de 'ubuntu' por el nombre de usuario que se tenga en la maquina. Ademas se debe borrar CMakeCache que tambien se encuentra en /build dentro de la carpeta del repositorio.

Una vez realizados estos cambios se podra ejecutar catkin_make con previo source del workspace. En caso de no entender este termino referirse al tutorial oficial de ROS de "create_a_workspace".


