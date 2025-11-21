#!/usr/bin/env python3
"""
Script pour configurer un ODrive en mode CAN
Usage: python3 configure_odrive_can.py <node_id>
"""

import odrive
from odrive.enums import *
import sys
import time

def configure_odrive_can(node_id):
    """Configure un ODrive pour le bus CAN"""
    
    if node_id < 0 or node_id > 5:
        print(f"❌ Node ID invalide: {node_id} (doit être entre 0 et 5)")
        return False
    
    print(f"\n{'='*60}")
    print(f"Configuration ODrive pour CAN Node ID {node_id}")
    print(f"{'='*60}\n")
    
    print("Recherche d'un ODrive connecté en USB...")
    try:
        odrv = odrive.find_any(timeout=15)
    except Exception as e:
        print(f"❌ Aucun ODrive trouvé: {e}")
        print("\nVérifiez:")
        print("  1. L'ODrive est allumé")
        print("  2. Le câble USB est connecté")
        print("  3. Un seul ODrive est connecté en USB à la fois")
        return False
    
    print(f"✅ ODrive trouvé: SN {odrv.serial_number}\n")
    
    # Afficher la configuration actuelle
    print("--- Configuration actuelle ---")
    print(f"CAN Enabled: {odrv.config.enable_can_a}")
    print(f"CAN Node ID: {odrv.can.node_id}")
    print(f"CAN Baud Rate: {odrv.can.config.baud_rate}")
    print(f"Axis 0 Watchdog: {odrv.axis0.config.enable_watchdog}")
    print(f"Axis 1 Watchdog: {odrv.axis1.config.enable_watchdog}")
    
    # Demander confirmation
    print(f"\n⚠️  Cette opération va:")
    print(f"  1. Configurer le CAN à 250000 baud")
    print(f"  2. Définir le Node ID à {node_id}")
    print(f"  3. Activer les watchdogs (timeout 0.1s)")
    print(f"  4. Sauvegarder et redémarrer l'ODrive")
    
    response = input(f"\nContinuer ? (oui/non): ").strip().lower()
    if response not in ['oui', 'o', 'yes', 'y']:
        print("❌ Opération annulée")
        return False
    
    print("\n--- Application de la configuration ---")
    
    try:
        # Configuration CAN
        print("1. Configuration du bus CAN...")
        odrv.config.enable_can_a = True
        odrv.can.config.baud_rate = 250000
        odrv.can.node_id = node_id
        print(f"   ✅ CAN activé, Node ID = {node_id}, Baud = 250000")
        
        # Configuration des watchdogs
        print("2. Configuration des watchdogs...")
        odrv.axis0.config.enable_watchdog = True
        odrv.axis0.config.watchdog_timeout = 0.1
        odrv.axis1.config.enable_watchdog = True
        odrv.axis1.config.watchdog_timeout = 0.1
        print("   ✅ Watchdogs activés (0.1s)")
        
        # Effacer les erreurs
        print("3. Effacement des erreurs...")
        odrv.clear_errors()
        print("   ✅ Erreurs effacées")
        
        # Sauvegarder
        print("4. Sauvegarde de la configuration...")
        odrv.save_configuration()
        print("   ✅ Configuration sauvegardée")
        
        print("\n5. Redémarrage de l'ODrive...")
        print("   (Attendez 5 secondes...)")
        try:
            odrv.reboot()
        except:
            pass  # La connexion sera perdue lors du reboot
        
        time.sleep(5)
        
        print("\n{'='*60}")
        print("✅ CONFIGURATION TERMINÉE !")
        print(f"{'='*60}")
        print(f"\nL'ODrive avec Node ID {node_id} est maintenant configuré.")
        print("\nProchaines étapes:")
        print("  1. Débranchez le câble USB de cet ODrive")
        print("  2. Connectez le prochain ODrive en USB")
        print("  3. Relancez ce script avec le prochain Node ID")
        print(f"     python3 configure_odrive_can.py {node_id + 1}")
        
        return True
        
    except Exception as e:
        print(f"\n❌ ERREUR lors de la configuration: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 configure_odrive_can.py <node_id>")
        print("\nExemples:")
        print("  python3 configure_odrive_can.py 0  # Configure le premier ODrive")
        print("  python3 configure_odrive_can.py 1  # Configure le deuxième ODrive")
        print("  python3 configure_odrive_can.py 2  # Configure le troisième ODrive")
        print("  ... etc jusqu'à 5")
        print("\n⚠️  Connectez UN SEUL ODrive en USB à la fois !")
        sys.exit(1)
    
    try:
        node_id = int(sys.argv[1])
    except ValueError:
        print(f"❌ Node ID invalide: {sys.argv[1]} (doit être un nombre entre 0 et 5)")
        sys.exit(1)
    
    success = configure_odrive_can(node_id)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrompu par l'utilisateur")
        sys.exit(0)







