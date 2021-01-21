# STM32Bicoptero

Este proyecto de STM32 trata del control para un bicóptero cuyo eje de rotación se encuentra en medio de los dos motores.

Para realizar este proyecto se requiere de 2 motores de dron, un rodamiento, helices complementarias para los motores y una base para poner la parte mecánica y la parte electrónica.

El proyecto fue realizado en un STM32F103C8T6 (Blue Pill), consta de conexión I2C con el módulo MPU6050 para obtener el ángulo de giro del eje, una conexión UART para enviar los datos por puerto serial al computador, la implementación de un filtro digital Butterworth de orden 8 (A partir de 4 sistemas de 2do orden).

El período de muestreo de los filtros y el control (aún no implementado) es de 0.01 segundos.

Utiliza el Timer 1 para hacer PWM y el timer 2 para hacer la interrupción repetitiva para el muestreo.
