#!/bin/bash
# Script pour mettre tous les ODrives en CLOSED_LOOP_CONTROL via CAN
# OpenDog v3 - 12 axes (Node IDs 0-11)

echo "=========================================="
echo "DÉMARRAGE CLOSED_LOOP_CONTROL via CAN"
echo "=========================================="

# Vérifier que CAN0 est UP
if ! ip link show can0 | grep -q "UP"; then
    echo "❌ Erreur: can0 n'est pas UP"
    echo "Exécutez: sudo ip link set can0 up type can bitrate 250000"
    exit 1
fi

echo "✅ Interface CAN0 active"
echo ""

# Commande ODrive: Set Axis State = 8 (CLOSED_LOOP_CONTROL)
# Format: cansend can0 <CMD_ID>#<DATA>
# CMD_ID = 0x007 + (node_id << 5)
# DATA = 08000000 (little-endian, requested_state = 8)

echo "Envoi des commandes CLOSED_LOOP_CONTROL..."

# Node ID 0 (0x007)
echo "  → Node 0"
cansend can0 007#0800000000000000

# Node ID 1 (0x027)
echo "  → Node 1"
cansend can0 027#0800000000000000

# Node ID 2 (0x047)
echo "  → Node 2"
cansend can0 047#0800000000000000

# Node ID 3 (0x067)
echo "  → Node 3"
cansend can0 067#0800000000000000

# Node ID 4 (0x087)
echo "  → Node 4"
cansend can0 087#0800000000000000

# Node ID 5 (0x0A7)
echo "  → Node 5"
cansend can0 0A7#0800000000000000

# Node ID 6 (0x0C7)
echo "  → Node 6"
cansend can0 0C7#0800000000000000

# Node ID 7 (0x0E7)
echo "  → Node 7"
cansend can0 0E7#0800000000000000

# Node ID 8 (0x107)
echo "  → Node 8"
cansend can0 107#0800000000000000

# Node ID 9 (0x127)
echo "  → Node 9"
cansend can0 127#0800000000000000

# Node ID 10 (0x147)
echo "  → Node 10"
cansend can0 147#0800000000000000

# Node ID 11 (0x167)
echo "  → Node 11"
cansend can0 167#0800000000000000

sleep 1

echo ""
echo "✅ Commandes envoyées !"
echo ""
echo "Vérification des états (heartbeat)..."
echo "Recherche de l'état 8 (CLOSED_LOOP) dans les messages..."
echo ""

# Capturer quelques heartbeats
candump can0 -n 30 2>&1 | grep "can0  0[0-9a-f][0-9a-f]   \[8\]" | head -12

echo ""
echo "=========================================="
echo "Si vous voyez '08 00 00 00' → ✅ SUCCÈS"
echo "Si vous voyez '01 00 00 00' → ❌ Encore IDLE"
echo "=========================================="




