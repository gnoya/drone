# Drone

## Controlador de vuelo

El drone utiliza tres controladores para cada uno de las variables a controlar: yaw rate, pitch y roll; cada uno de ellos cuenta con un PID, y los tres actúan al mismo tiempo sobre los cuatro motores a una frecuencia de 100 Hz.

### Procesadores
Se utilizaron dos procesadores: un Teensy 3.2 para la lectura de sensores, control remoto, cálculo de errores de variables, cálculo de salidas de los controladores y actuador sobre los motores, y un Raspberry Pi 3 para ejecutar ROS y el servidor que permite ejecutar la interfaz web en una computadora por wifi.

### IMU

La IMU utilizada es el sensor MPU6050, cuyo DMP permite leer la posición del drone a una frecuencia de 100 Hz; dicho DMP realiza una fusión de datos entre su acelerómetro y su giroscópio, por lo que sus resultados son muy precisos. Se comunica al procesador utilizando el protocolo de comunicación I2C. 

## Interfaz gráfica web

Interfaz web conectada con el drone a través de ROS, programada utilizando HTML, javascript y CSS. Para las gráficas en tiempo real, se utilizó la librería [Rickshaw](https://github.com/shutterstock/rickshaw).

![alt text](https://github.com/gnoya/drone/blob/master/web/src/img/ui.png)

En las gráficas, la línea azul corresponde al setpoint de cada ángulo a controlar, y la línea negra corresponde al valor medido por el giroscopio. El switche "Motor" apaga y/o prende los motores, y se puede utilizar para detener al drone en caso de emergencia. El botón "Reboot" reinicia la comunicación I2C con el giroscopio, en caso de notar que éste se pierde (puede pasar cuando se golpea el drone).

Las constantes de los PID son editables con un click, y se actualizan automáticamente en el drone sin necesidad de reiniciarlo. En la figura del drone se puede observar el porcentaje de potencia asignado a cada uno de los motores. Todo lo mencionado se realiza en tiempo real.

### TODO:
Terminar el paquete de ROS.

Ajustar las constantes de los PID utilizando unbanco de prueba.

Migrar ROS a Raspberry Pi.
