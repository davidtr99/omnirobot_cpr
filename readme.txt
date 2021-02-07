------------------------------------------------------------------------
                            PROYECTO DE CPR
            Control de robot omnidireccional basasado en percepción
------------------------------------------------------------------------

------------------------------------------------------------------------
Autores:
---------------
José Manuel González Marín
David Tejero Ruiz
Miguel Granero Ramos
Maria Castro Soto
Pablo Díaz Rodríguez

------------------------------------------------------------------------
Descripción
---------------
Este comprimido incluye 6 paquetes:
- camera_description    : incluye el modelo de la cámara para gazebo
- nexus_description     : describe el modelo de alto nivel que controlamos para gazebo
- omnirobot_control     : contiene los ejecutables Control.py y dosificador.py
- percepcion            : incluye los ejecutables mapeado.py y robotPosition.py
- planificador          : incluye el ejecutable planificador.py
------------------------------------------------------------------------
Guia de usuario
---------------
1. Incluir todos los paquetes en la carpeta src del workSpace (ej. catkin_ws/src). Hacer catkin_make.

2. Abrir una terminal y ejecutar: roslaunch proyecto_gazebo proyecto.launch
[El proyecto entero se debe iniciar sin errores]

3. En la ventana "Seleccion_punto_final" hacer click sobre el punto final deseado
[Se planifica una trayectoria y el robot empieza a moverse, abriéndose una nueva ventana que muestra la trayectoria]
[Puede modificarse el tiempo de ejecución el mapa y observar como replanifica]
[Se puede pulsar en cualquier momento oto punto destino para planificar otra trayectoria]

4. Se puede cambiar el angulo de orientación del robot con la siguiente orden:
rosparam set /angle_robot xx
[Siendo xx el ángulo a fijar para el robot entre 0 y 360º]
