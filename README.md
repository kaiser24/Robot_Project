# Robot_Project

En este proyecto controlamos un brazo robotico de 4 grados de libertad por medio de un programa escrito en python que envia ordenes por medio del puerto serial a un microcontrolador (Arduino) y este a su vez controla los servomotores de las articulaciones del robot. Por medio de una camara (De celular en este caso por medio de laaplicación IpWebcam) detectamos la posición de un objeto en el area de trabajo y se envía esta posición al robot para que recoja el objeto y lo re-ubique.

#### Cinematica Inversa
Para controlar el robot simplemente con posiciones cartesianas fué necesario obtener las ecuaciones de la cinematica inversa para los angulos de las articulaciones.
Se empleó el metodo geometrico.

#### Detección del Objeto.
Para la detección del objeto, primero capturamos la imagen empleando la aplicación de móvil IpWebcam, y se empleó la libreria de visión artificial OpenCV para procesar la imagen.
En resumen, para detectar el objeto se trata se quitar sombras sobre el entorno, se aplica un umbral sobre la imagen para solo dejar los objetos en la imagen, se detectan los objetos que haya en la macara resultante y solo se toma el de mayor tamaño.

## Indicaciones
El archivo Robot_Control.py es el programa en python para efectura el control del robot y la deteccion de la imagen.

El archivo robot.ino dentro de la carpeta contiene el programa en arduino que recibe las ordenes de python por serial.

##### Los 3 archivos restantes son solo de prueba
Project_IK_4DOF.py es solo para probar que la cinematica inversa esté funcionando correctamente.

object_detection_robot.py es solo para probar la detección en la imagen por video.

photo_object_detection_robot.py es solo para probar la detección en la imagen con 1 sola captura.
