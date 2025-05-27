#!/bin/bash

# Verifica si 'fuser' está instalado, y si no, lo instala
if ! command -v fuser >/dev/null 2>&1; then
    echo "'fuser' no está instalado. Instalando 'psmisc'..."
    if command -v apt-get >/dev/null 2>&1; then
        sudo apt-get update
        sudo apt-get install -y psmisc
    elif command -v yum >/dev/null 2>&1; then
        sudo yum install -y psmisc
    else
        echo "No se pudo determinar el gestor de paquetes. Instala 'psmisc' manualmente."
        exit 1
    fi
fi

# Lista de dispositivos a liberar
DEVICES=("/dev/video0" "/dev/video4" "/dev/ttyACM0" "/dev/ttyACM1")

for dev in "${DEVICES[@]}"; do
    echo "Checking processes using $dev"
    fuser -k "$dev"
done

echo "All matching processes killed."

