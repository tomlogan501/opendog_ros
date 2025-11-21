#!/bin/bash
# Script pour tester le parsing des messages 0x009

echo "=========================================="
echo "TEST DU PARSING DES MESSAGES 0x009"
echo "=========================================="
echo ""

echo "1. Capture des messages CAN pendant 5 secondes..."
echo "   Recherche de messages 0x009 avec DONNÉES (8 bytes)"
echo ""

# Capturer les messages pendant 5 secondes
timeout 5 candump can0 2>&1 > /tmp/can_capture.txt

echo "2. Analyse des messages 0x009..."
echo ""

# Chercher les messages 0x009 (tous les nodes)
echo "=== Requêtes 0x009 (0 bytes) ==="
grep -E "can0  0[0-9a-fA-F]9   \[0\]" /tmp/can_capture.txt | head -20

echo ""
echo "=== Réponses 0x009 (8 bytes) ==="
grep -E "can0  0[0-9a-fA-F]9   \[8\]" /tmp/can_capture.txt | head -20

echo ""
echo "=========================================="
echo "ANALYSE:"
echo "  - Si vous voyez [0] → Requêtes envoyées ✅"
echo "  - Si vous voyez [8] → Réponses reçues ✅✅"
echo "  - Si AUCUN [8] → ODrives ne répondent ❌"
echo "=========================================="
echo ""

# Compter les messages
req_count=$(grep -c -E "can0  0[0-9a-fA-F]9   \[0\]" /tmp/can_capture.txt)
resp_count=$(grep -c -E "can0  0[0-9a-fA-F]9   \[8\]" /tmp/can_capture.txt)

echo "📊 STATISTIQUES:"
echo "   Requêtes 0x009: $req_count"
echo "   Réponses 0x009: $resp_count"
echo ""

if [ $resp_count -eq 0 ]; then
    echo "❌ AUCUNE réponse ! Les ODrives ne sont pas en CLOSED_LOOP"
    echo ""
    echo "🔧 SOLUTION:"
    echo "   1. Vérifier que les ODrives sont en CLOSED_LOOP:"
    echo "      candump can0 -n 20 | grep 'can0  0[0-9a-f]1'"
    echo "   2. Chercher l'état '08' (CLOSED_LOOP) dans byte 5"
    echo "   3. Si état '01' (IDLE), exécuter:"
    echo "      /home/dev/opendog_ws/scripts/start_closed_loop_can.sh"
else
    echo "✅ Les ODrives répondent ! Le parsing fonctionne !"
    echo ""
    echo "📋 Détail des réponses:"
    grep -E "can0  0[0-9a-fA-F]9   \[8\]" /tmp/can_capture.txt | while read line; do
        can_id=$(echo $line | awk '{print $2}')
        node_id=$((0x$can_id >> 5))
        echo "   Node $node_id répond"
    done
fi




