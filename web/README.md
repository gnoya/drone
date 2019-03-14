# Osciloscopio

En esta carpeta no se encuentran los módulos de NPM necesarios para funcionar. Estos módulos deben ser descargados desde el terminal. Esto se hizo para no subir al repositorio todo el peso de los mismos.

# server.js

Este es nuestro "main", es el archivo que se ejecuta para montar el servidor web que correrá el osciloscopio en un navegador. Para la lectura del puerto serial se utilizó el módulo "serialport" de npm, y para la lectura rápida del puerto se utilizó el módulo nanotimer. Este módulo se utilizó para colocar un intervalo de microsegundos para la correcta lectura del puerto.

# test.js

Este archivo se utilizó para probar la interfaz gráfica colocando un "generador de funciones virtual" como entrada.

# Carpeta public:
En esta carpeta se encuentran los archivos HTML y JS para el funcionamiento de la pagina web. Para la interfaz grafica utilizamos una librería llamada P5.js, que permite dibujar sobre un canvas de una manera sencilla.

El servidor web se implementó utilizando Express. La comunicación entre la página web y el servidor web se realizó a través de sockets, utilizando el módulo Socket.io.

Autores:

Gabriel Noya 13-10982
Domingo Luis 13-10773