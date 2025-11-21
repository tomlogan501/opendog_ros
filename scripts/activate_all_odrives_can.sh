#!/bin/bash

# Script pour activer tous les ODrives en CLOSED_LOOP_CONTROL via CAN
# Node IDs: 0 à 11

echo "═══════════════════════════════════════════════════════════"
echo "  Activation de tous les ODrives en CLOSED_LOOP_CONTROL"
echo "═══════════════════════════════════════════════════════════"

# Vérifier que can0 est UP
if ! ip link show can0 | grep -q "UP"; then
    echo "❌ Erreur : can0 n'est pas UP"
    exit 1
fi

echo ""
echo "✅ Bus CAN actif"
echo ""

# Fonction pour envoyer une commande CAN
send_can_command() {
    local node_id=$1
    local cmd_id=$2
    local data=$3
    local can_id=$((node_id * 32 + cmd_id))
    
    cansend can0 $(printf "%03X" $can_id)#$data
}

# CMD 0x07 : Set Axis State
# Data : 08 00 00 00 (AXIS_STATE_CLOSED_LOOP_CONTROL = 8)

echo "📤 Envoi de la commande CLOSED_LOOP_CONTROL à tous les ODrives..."
echo ""

for node_id in {0..11}; do
    echo "  → Node ID $node_id : Set Axis State = 8 (CLOSED_LOOP_CONTROL)"
    send_can_command $node_id 7 "08000000"
    sleep 0.1
done

echo ""
echo "✅ Commandes envoyées !"
echo ""
echo "⏳ Attente de 2 secondes pour la stabilisation..."
sleep 2

echo ""
echo "📊 Vérification des heartbeats (Ctrl+C pour arrêter)..."
echo ""
timeout 5 candump can0 | grep " 001 " | head -12

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  Vérifiez que les axes sont en state 8 (CLOSED_LOOP)"
echo "  Byte 4 du heartbeat doit être 08"
echo "═══════════════════════════════════════════════════════════"

