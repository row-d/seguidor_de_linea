#ifndef BLUETOOTH_CONFIG_H
#define BLUETOOTH_CONFIG_H

// Nombre visible del HC-05: sin espacios ni acentos.
const char BLUETOOTH_NAME[] = "RobotSeguidor";
const unsigned long BLUETOOTH_BAUD = 9600;

// En true, el sketch solo envia esta configuracion al HC-05 y muestra el resultado por USB.
// El modulo debe estar en modo AT: ver "Configurar el Bluetooth" en el README.
const bool BLUETOOTH_SETUP_MODE = false;

#endif
