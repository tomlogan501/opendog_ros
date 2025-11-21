#!/usr/bin/env python3
"""
Script pour configurer tous les ODrives avec des limites de courant réduites
pour fonctionner avec une alimentation limitée (24V 10A = 240W)
"""

import odrive
from odrive.enums import *
import sys
import time

def configure_odrive_low_current(odrv, node_ids):
    """
    Configure un ODrive avec des limites de courant très basses
    
    Args:
        odrv: Instance ODrive
        node_ids: Tuple (axis0_node_id, axis1_node_id)
    """
    print(f"\n{'='*60}")
    print(f"Configuration ODrive - Node IDs: {node_ids[0]} et {node_ids[1]}")
    print(f"{'='*60}")
    
    # Effacer les erreurs
    odrv.clear_errors()
    time.sleep(0.1)
    
    # Configuration Axis 0
    print(f"\nConfiguration Axis 0 (Node ID {node_ids[0]})...")
    odrv.axis0.motor.config.current_lim = 2.0  # 2A max
    odrv.axis0.motor.config.current_lim_margin = 2.0
    odrv.axis0.motor.config.calibration_current = 2.0
    odrv.axis0.motor.config.requested_current_range = 10.0
    
    # Configuration Axis 1
    print(f"Configuration Axis 1 (Node ID {node_ids[1]})...")
    odrv.axis1.motor.config.current_lim = 2.0  # 2A max
    odrv.axis1.motor.config.current_lim_margin = 2.0
    odrv.axis1.motor.config.calibration_current = 2.0
    odrv.axis1.motor.config.requested_current_range = 10.0
    
    # Configuration CAN (vérification)
    print(f"\nVérification configuration CAN...")
    print(f"  CAN enabled: {odrv.config.enable_can_a}")
    print(f"  Baud rate: {odrv.can.config.baud_rate}")
    print(f"  Axis 0 node_id: {odrv.axis0.config.can.node_id}")
    print(f"  Axis 1 node_id: {odrv.axis1.config.can.node_id}")
    
    # Configuration du mode de contrôle
    print(f"\nConfiguration du mode de contrôle...")
    odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    odrv.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    odrv.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    
    # Watchdog (déjà configuré normalement)
    print(f"\nVérification Watchdog...")
    print(f"  Axis 0 watchdog: {odrv.axis0.config.enable_watchdog}, timeout: {odrv.axis0.config.watchdog_timeout}")
    print(f"  Axis 1 watchdog: {odrv.axis1.config.enable_watchdog}, timeout: {odrv.axis1.config.watchdog_timeout}")
    
    # Sauvegarder et rebooter
    print(f"\n💾 Sauvegarde de la configuration...")
    odrv.save_configuration()
    
    print(f"🔄 Redémarrage de l'ODrive...")
    try:
        odrv.reboot()
    except:
        pass  # La connexion est perdue pendant le reboot
    
    print(f"✅ Configuration terminée pour Node IDs {node_ids[0]} et {node_ids[1]}")
    print(f"⏳ Attendez 5 secondes pour le reboot...")
    time.sleep(5)


def verify_odrive_config(odrv, node_ids):
    """Vérifie la configuration après reboot"""
    print(f"\n{'='*60}")
    print(f"Vérification ODrive - Node IDs: {node_ids[0]} et {node_ids[1]}")
    print(f"{'='*60}")
    
    print(f"\n📊 État actuel:")
    print(f"  VBUS: {odrv.vbus_voltage:.2f} V")
    print(f"  Axis 0 current_lim: {odrv.axis0.motor.config.current_lim} A")
    print(f"  Axis 1 current_lim: {odrv.axis1.motor.config.current_lim} A")
    print(f"  Axis 0 state: {odrv.axis0.current_state}")
    print(f"  Axis 1 state: {odrv.axis1.current_state}")
    print(f"  Axis 0 error: {hex(odrv.axis0.error)}")
    print(f"  Axis 1 error: {hex(odrv.axis1.error)}")
    
    # Tester CLOSED_LOOP_CONTROL
    print(f"\n🧪 Test CLOSED_LOOP_CONTROL...")
    odrv.clear_errors()
    time.sleep(0.1)
    
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(2)
    
    print(f"  Axis 0 state après commande: {odrv.axis0.current_state}")
    print(f"  Axis 1 state après commande: {odrv.axis1.current_state}")
    print(f"  Axis 0 error: {hex(odrv.axis0.error)}")
    print(f"  Axis 1 error: {hex(odrv.axis1.error)}")
    
    if odrv.axis0.current_state == 8 and odrv.axis1.current_state == 8:
        print(f"  ✅ Les deux axes sont en CLOSED_LOOP_CONTROL !")
    else:
        print(f"  ❌ Échec du passage en CLOSED_LOOP_CONTROL")
        print(f"  Axis 0 motor error: {hex(odrv.axis0.motor.error)}")
        print(f"  Axis 1 motor error: {hex(odrv.axis1.motor.error)}")
    
    # Remettre en IDLE
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE


def main():
    # Mapping des node IDs (axis0, axis1) pour chaque ODrive
    odrive_node_mapping = [
        (0, 1),    # ODrive 1
        (2, 3),    # ODrive 2
        (4, 5),    # ODrive 3
        (6, 7),    # ODrive 4
        (8, 9),    # ODrive 5
        (10, 11),  # ODrive 6
    ]
    
    print("="*60)
    print("CONFIGURATION DES ODRIVES AVEC COURANT RÉDUIT")
    print("="*60)
    print("\n⚠️  IMPORTANT:")
    print("  - Connectez UN SEUL ODrive à la fois en USB")
    print("  - Courant limite: 2A par moteur")
    print("  - Puissance totale: ~50W par ODrive (2 moteurs)")
    print("  - Avec 6 ODrives: ~300W total (votre alim: 240W max)")
    print("\n⚠️  VOTRE ALIMENTATION EST TOUJOURS INSUFFISANTE!")
    print("  - Recommandation: Tester avec 4 ODrives max (8 moteurs)")
    print("  - Ou réduire encore le courant à 1A par moteur")
    print("\n")
    
    for i, node_ids in enumerate(odrive_node_mapping, 1):
        print(f"\n{'#'*60}")
        print(f"# ODrive {i}/6 - Node IDs: {node_ids[0]} et {node_ids[1]}")
        print(f"{'#'*60}")
        
        input(f"\n📌 Connectez l'ODrive {i} en USB et appuyez sur ENTRÉE...")
        
        print(f"\n🔍 Recherche de l'ODrive...")
        try:
            odrv = odrive.find_any()
            print(f"✅ ODrive trouvé: {odrv.serial_number}")
            
            # Vérifier que les node IDs correspondent
            actual_node0 = odrv.axis0.config.can.node_id
            actual_node1 = odrv.axis1.config.can.node_id
            
            if (actual_node0, actual_node1) != node_ids:
                print(f"⚠️  ATTENTION: Node IDs ne correspondent pas!")
                print(f"   Attendu: {node_ids}")
                print(f"   Trouvé: ({actual_node0}, {actual_node1})")
                response = input("Continuer quand même? (o/n): ")
                if response.lower() != 'o':
                    print("Passé.")
                    continue
            
            # Configurer
            configure_odrive_low_current(odrv, node_ids)
            
            # Reconnecter après reboot
            print(f"\n🔍 Reconnexion après reboot...")
            odrv = odrive.find_any()
            
            # Vérifier
            verify_odrive_config(odrv, node_ids)
            
        except Exception as e:
            print(f"❌ Erreur: {e}")
            response = input("Continuer avec le prochain ODrive? (o/n): ")
            if response.lower() != 'o':
                sys.exit(1)
    
    print(f"\n{'='*60}")
    print("✅ CONFIGURATION TERMINÉE POUR TOUS LES ODRIVES")
    print(f"{'='*60}")
    print("\n📋 Résumé:")
    print("  - Courant limite: 2A par moteur")
    print("  - 12 moteurs × 2A = 24A total")
    print("  - Puissance théorique: 24V × 24A = 576W")
    print("\n⚠️  VOTRE ALIMENTATION (240W) EST INSUFFISANTE!")
    print("\n💡 Solutions:")
    print("  1. Tester avec 4 ODrives seulement (débrancher 2)")
    print("  2. Acheter une alimentation 24V 30A minimum")
    print("  3. Utiliser plusieurs alimentations en parallèle")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interruption par l'utilisateur")
        sys.exit(0)



