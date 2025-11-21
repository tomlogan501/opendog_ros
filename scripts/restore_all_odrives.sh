#!/bin/bash
# Script pour restaurer la configuration de toutes les cartes ODrive

echo "🔄 RESTAURATION DES CONFIGURATIONS ODRIVE"
echo "=========================================="
echo ""
echo "⚠️  INSTRUCTIONS :"
echo "   1. Branchez UNE SEULE carte ODrive à la fois en USB"
echo "   2. Appuyez sur ENTRÉE pour restaurer sa configuration"
echo "   3. Débranchez-la et branchez la suivante"
echo ""

# Liste des fichiers de backup
declare -a configs=(
    "/home/divin/my_odrive_config_card1.json"
    "/home/divin/my_odrive_config_card2.json"
    "/home/divin/my_odrive_config_card3.json"
    "/home/divin/my_odrive_config_card4.json"
    "/home/divin/my_odrive_config_card5.json"
    "/home/divin/my_odrive_config_card6.json"
)

for i in "${!configs[@]}"; do
    card_num=$((i + 1))
    config_file="${configs[$i]}"
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📋 CARTE $card_num / 6"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "1️⃣  Branchez la CARTE $card_num en USB"
    echo "2️⃣  Appuyez sur ENTRÉE pour restaurer..."
    read -p ""
    
    echo ""
    echo "🔍 Recherche de l'ODrive..."
    if ! odrivetool shell -c "print('ODrive trouvé')" 2>/dev/null; then
        echo "❌ Aucun ODrive détecté !"
        echo "   → Vérifiez la connexion USB"
        echo "   → Appuyez sur ENTRÉE pour réessayer..."
        read -p ""
        continue
    fi
    
    echo "✅ ODrive détecté !"
    echo ""
    echo "🔄 Restauration de la configuration..."
    echo "   Fichier : $config_file"
    
    if odrivetool restore-config "$config_file"; then
        echo ""
        echo "✅✅✅ CARTE $card_num RESTAURÉE AVEC SUCCÈS !"
        echo ""
        echo "📊 Vérification de la configuration..."
        odrivetool shell << 'EOF'
import odrive
odrv0 = odrive.find_any()
print(f"\n=== CONFIGURATION RESTAURÉE ===")
print(f"Serial: {odrv0.serial_number}")
print(f"CAN baud rate: {odrv0.can.config.baud_rate}")
print(f"Axis0 node_id: {odrv0.axis0.config.can.node_id}")
print(f"Axis1 node_id: {odrv0.axis1.config.can.node_id}")
print(f"Axis0 motor calibrated: {odrv0.axis0.motor.config.pre_calibrated}")
print(f"Axis0 encoder ready: {odrv0.axis0.encoder.is_ready}")
print(f"Axis1 motor calibrated: {odrv0.axis1.motor.config.pre_calibrated}")
print(f"Axis1 encoder ready: {odrv0.axis1.encoder.is_ready}")
print(f"\n✅ Carte $card_num prête à l'emploi !")
EOF
    else
        echo ""
        echo "❌ ÉCHEC DE LA RESTAURATION"
        echo "   → Vérifiez que le fichier existe"
        echo "   → Vérifiez que l'ODrive est bien connecté"
    fi
    
    if [ $card_num -lt 6 ]; then
        echo ""
        echo "➡️  Débranchez la CARTE $card_num et branchez la CARTE $((card_num + 1))"
        echo "    Appuyez sur ENTRÉE pour continuer..."
        read -p ""
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎉 RESTAURATION TERMINÉE !"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "✅ Les 6 cartes ODrive ont été restaurées"
echo ""
echo "📋 PROCHAINES ÉTAPES :"
echo "   1. Débranchez l'USB de toutes les cartes"
echo "   2. Vérifiez que le bus CAN est actif (can0)"
echo "   3. Lancez le hardware layer ROS2"
echo ""






