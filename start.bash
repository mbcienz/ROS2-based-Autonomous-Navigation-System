#!/bin/bash

# Funzione per creare una nuova scheda con comando
create_tab() {
    local title="$1"
    local command="$2"
    gnome-terminal --tab --title="$title" -- bash -c "source install/setup.bash && $command; exec bash"
}

echo "Avvio dei servizi ROS2 su schede separate..."

# Prima scheda - Localization
create_tab "Localization" "ros2 launch navigation_pkg localization.launch.py"

sleep 2

# Seconda scheda - Navigation Server
create_tab "Navigation Server" "ros2 launch navigation_pkg navigation_server.launch.py"

sleep 2

# Terza scheda - Perception
create_tab "Perception" "ros2 launch perception_pkg perception.launch.py"

sleep 2

# Quarta scheda - Navigation
create_tab "Navigation" "ros2 launch navigation_pkg navigation.launch.py"

sleep 2

# Quinta scheda - TurtleBot4 Visualization
create_tab "TurtleBot4 Viz" "ros2 launch turtlebot4_viz view_robot.launch.py"

# Aspetta un momento prima di aprire la scheda successiva
sleep 2

echo "Tutti i servizi ROS2 sono stati avviati in schede separate!"
echo "Lo script terminerà automaticamente tra 3 secondi..."

# Countdown prima di chiudere
for i in 3 2 1; do
    echo "Chiusura in $i..."
    sleep 1
done

echo "Script completato. La finestra si chiuderà ora."
exit 0
