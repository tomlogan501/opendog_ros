#!/bin/bash
# Script pour tester le hardware layer en mode READ
# Ce script permet de vérifier que les messages CAN sont parsés correctement

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 TEST DU HARDWARE LAYER EN MODE READ"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Vérifier que le bus CAN est actif
echo "1️⃣  Vérification du bus CAN..."
if ! ip link show can0 | grep -q "UP"; then
    echo "❌ Le bus CAN n'est pas actif !"
    echo "   Configuration du bus CAN..."
    sudo ip link set can0 down 2>/dev/null
    sudo ip link set can0 type can bitrate 250000 restart-ms 100
    sudo ip link set can0 txqueuelen 1000
    sudo ip link set can0 up
    echo "✅ Bus CAN configuré (250 kbps)"
else
    echo "✅ Bus CAN actif"
fi

# Afficher les paramètres CAN
echo ""
echo "📊 Paramètres CAN :"
ip -details link show can0 | grep -E "can|bitrate|txqueue"

# Vérifier les messages CAN
echo ""
echo "2️⃣  Vérification des messages CAN (5 secondes)..."
echo "   (Si vous voyez des messages, les ODrives communiquent)"
echo ""
timeout 5 candump can0 | head -20

if [ $? -eq 124 ]; then
    echo ""
    echo "✅ Capture terminée"
else
    echo ""
    echo "❌ Aucun message CAN détecté !"
    echo "   → Vérifiez que les ODrives sont en CLOSED_LOOP_CONTROL"
    echo "   → Vérifiez les connexions CAN (CANH, CANL, GND)"
    exit 1
fi

echo ""
echo "3️⃣  Lancement du hardware layer ROS2..."
echo "   (Les logs seront enregistrés dans /tmp/opendog_read_test.log)"
echo ""

# Source ROS2
cd /home/dev/opendog_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Lancer le hardware layer et enregistrer les logs
echo "🚀 Démarrage..."
echo ""
ros2 launch opendog_bringup opendog_bringup_can.launch.py 2>&1 | tee /tmp/opendog_read_test.log






