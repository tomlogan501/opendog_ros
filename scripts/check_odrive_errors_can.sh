#!/bin/bash
# Script pour vérifier les erreurs des ODrives via CAN
# Lit les messages Get_Error sur le bus CAN

echo "=========================================="
echo "VÉRIFICATION ERREURS ODRIVES via CAN"
echo "=========================================="
echo ""

# Fonction pour demander les erreurs d'un node
request_errors() {
    local node_id=$1
    local cmd_id=$(printf "%03X" $((0x003 + (node_id << 5))))
    
    echo "Node $node_id (CMD: 0x$cmd_id):"
    
    # Envoyer Get_Error (CMD 0x003)
    cansend can0 ${cmd_id}#
    
    # Attendre la réponse (0.1 sec)
    sleep 0.1
}

echo "Demande des erreurs pour tous les nodes..."
echo ""

# Demander les erreurs pour les nodes 0-11
for node in {0..11}; do
    request_errors $node
done

echo ""
echo "Capture des réponses (3 secondes)..."
echo ""

# Capturer les messages pendant 3 secondes
timeout 3 candump can0 2>&1 | grep -E "can0  0[0-9a-fA-F]{2}   \[8\]"

echo ""
echo "=========================================="
echo "ANALYSE:"
echo "  - Heartbeat (0x001): État de l'axe"
echo "  - Get_Error (0x003): Codes d'erreur"
echo "  - 00 00 00 00 = Pas d'erreur"
echo "  - Autre valeur = Erreur présente"
echo "=========================================="




