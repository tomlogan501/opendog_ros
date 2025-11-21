#!/bin/bash

# Script pour activer TOUS les ODrives en CLOSED_LOOP_CONTROL via CAN
# Node IDs: 0 à 11 (12 axes au total)

echo "=========================================="
echo "ACTIVATION DE TOUS LES ODRIVES EN CLOSED_LOOP_CONTROL"
echo "=========================================="
echo ""

# Fonction pour envoyer Set_Axis_State = CLOSED_LOOP_CONTROL (8)
send_closed_loop() {
    local node_id=$1
    local cmd_id=0x07  # Set_Axis_State
    local can_id=$((node_id * 32 + cmd_id))
    local can_id_hex=$(printf "0x%03X" $can_id)
    
    # État CLOSED_LOOP_CONTROL = 8 (0x08000000 en little-endian sur 4 octets)
    local data="08 00 00 00"
    
    echo "Node ID $node_id : Envoi Set_Axis_State = CLOSED_LOOP_CONTROL (8)"
    cansend can0 ${can_id_hex}#${data}
    
    # Petit délai pour éviter de saturer le bus
    sleep 0.1
}

echo "Envoi des commandes Set_Axis_State = CLOSED_LOOP_CONTROL..."
echo ""

# Boucle pour tous les Node IDs (0 à 11)
for node_id in {0..11}; do
    send_closed_loop $node_id
done

echo ""
echo "=========================================="
echo "COMMANDES ENVOYÉES !"
echo "=========================================="
echo ""
echo "Vérification dans 2 secondes..."
sleep 2

echo ""
echo "=========================================="
echo "VÉRIFICATION DES HEARTBEATS (5 secondes)"
echo "=========================================="
echo ""
echo "Recherche des Heartbeat (CAN ID 0x001, 0x021, 0x041, etc.)"
echo "Format attendu : [8] XX XX 00 00 08 00 00 00"
echo "                              ^^-- État 8 = CLOSED_LOOP"
echo ""

# Capture 50 messages et filtre les Heartbeat
candump can0 -n 50 | grep -E "0[0-9A-F]1   \[8\]"

echo ""
echo "=========================================="
echo "VÉRIFICATION DES /joint_states"
echo "=========================================="
echo ""
echo "Si les ODrives sont en CLOSED_LOOP, vous devriez voir des valeurs"
echo "au lieu de '.nan' pour tous les joints."
echo ""
echo "Exécutez maintenant :"
echo "  ros2 topic echo /joint_states --once"
echo ""

