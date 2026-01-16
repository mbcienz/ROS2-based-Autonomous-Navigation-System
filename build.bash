#!/bin/bash

# Script per lanciare i comandi ROS2 su schede diverse dello stesso terminale
# Funziona con gnome-terminal

echo "Pulizia delle cartelle di build precedenti..."

# Rimuovi le cartelle build, log e install se esistono
if [ -d "build" ]; then
    echo "Rimozione cartella build..."
    rm -rf build
fi

if [ -d "log" ]; then
    echo "Rimozione cartella log..."
    rm -rf log
fi

if [ -d "install" ]; then
    echo "Rimozione cartella install..."
    rm -rf install
fi

echo "Build del workspace con colcon..."
colcon build --symlink-install

# Controlla se il build è riuscito
if [ $? -ne 0 ]; then
    echo "ERRORE: Il build con colcon è fallito!"
    echo "Controlla gli errori sopra e riprova."
    exit 1
fi

echo "Build completato con successo!"
echo ""