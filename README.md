# thereminus
Arduino Theremin using Ultrasonic Sensors

## Configuración para prototipos actuales

Pines Arduino:

`0`: trigger del sensor ultrasonido  
`1`: echo del sensor ultrasonido  

`2` al `7`: sección del cubo de leds (según conexión de cubo de leds rojo, los cables son (de 2 a 7): azul, violeta. gris, blanco, negro, marrón)

`9`: botón pulsador

`10`: salida pwm para los parlantes (naranja)

`A0` al `A5`: sección del cubo de leds (según conexión de cubo de leds rojo, los cables son (de A0 a A5): negro, marrón, rojo, naranja, amarillo, verde)

`GND`: ground común de placa de alimentación (tira de pines hembra)  
`5V`: alimentación común de placa de alimentación (tira de pines hembra)


## Configuración para teclado picopico

Pines Arduino:

`A0` al `A4`: sección corta de cables de teclado (rojo, naranja, amarillo, verde, azul)  
`GND`: ground para el teclado (blanco)

`0` al `7`: sección larga de cables (gris, violeta, azul, verde, amarillo, narangja, rojo, marrón)

`10`: salida pwm para los parlantes de teclado (marrón)

`GND` : ground para los parlantes (rojo)
