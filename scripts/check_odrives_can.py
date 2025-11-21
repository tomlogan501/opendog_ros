#!/usr/bin/env python3
"""
Script de diagnostic pour vérifier la configuration CAN de tous les ODrives
"""

import odrive
from odrive.enums import *
import sys

def check_odrive_can(odrv, index):
    """Vérifie la configuration CAN d'un ODrive"""
    print(f"\n{'='*60}")
    print(f"ODrive #{index}")
    print(f"{'='*60}")
    
    try:
        # Informations de base
        print(f"Serial Number: {odrv.serial_number}")
        print(f"Hardware Version: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
        print(f"Firmware Version: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        
        # Configuration CAN
        print(f"\n--- Configuration CAN ---")
        print(f"CAN Enabled: {odrv.config.enable_can_a}")
        print(f"CAN Node ID: {odrv.can.node_id}")
        print(f"CAN Baud Rate: {odrv.can.config.baud_rate}")
        
        # État des axes
        print(f"\n--- État des Axes ---")
        for axis_num in [0, 1]:
            axis = getattr(odrv, f"axis{axis_num}")
            print(f"\nAxis {axis_num}:")
            print(f"  Current State: {axis.current_state}")
            print(f"  Axis Error: 0x{axis.error:08X}")
            print(f"  Motor Error: 0x{axis.motor.error:08X}")
            print(f"  Encoder Error: 0x{axis.encoder.error:08X}")
            print(f"  Controller Error: 0x{axis.controller.error:08X}")
            print(f"  Watchdog Enabled: {axis.config.enable_watchdog}")
            print(f"  Watchdog Timeout: {axis.config.watchdog_timeout}s")
        
        # Recommandations
        print(f"\n--- Diagnostic ---")
        issues = []
        
        if not odrv.config.enable_can_a:
            issues.append("❌ CAN n'est PAS activé !")
        else:
            print("✅ CAN est activé")
        
        if odrv.can.config.baud_rate != 250000:
            issues.append(f"⚠️  Baud rate incorrect: {odrv.can.config.baud_rate} (devrait être 250000)")
        else:
            print("✅ Baud rate correct (250000)")
        
        if odrv.can.node_id < 0 or odrv.can.node_id > 5:
            issues.append(f"⚠️  Node ID hors limites: {odrv.can.node_id} (devrait être 0-5)")
        else:
            print(f"✅ Node ID valide: {odrv.can.node_id}")
        
        if issues:
            print("\n🔴 PROBLÈMES DÉTECTÉS:")
            for issue in issues:
                print(f"  {issue}")
            return False
        else:
            print("\n✅ Configuration CAN correcte !")
            return True
            
    except Exception as e:
        print(f"❌ ERREUR lors de la lecture: {e}")
        return False

def main():
    print("="*60)
    print("DIAGNOSTIC DES ODRIVES - Configuration CAN")
    print("="*60)
    
    print("\nRecherche de tous les ODrives connectés...")
    print("(Cela peut prendre 10-30 secondes...)\n")
    
    # Trouver tous les ODrives
    odrives = []
    try:
        # Méthode 1 : Recherche générale
        print("Recherche en cours...")
        odrv = odrive.find_any(timeout=10)
        if odrv:
            odrives.append(odrv)
            print(f"✅ ODrive trouvé : SN {odrv.serial_number}")
            
            # Essayer de trouver d'autres ODrives
            for i in range(5):  # Chercher jusqu'à 5 autres
                try:
                    print(f"Recherche d'un autre ODrive...")
                    odrv = odrive.find_any(timeout=5)
                    if odrv and odrv not in odrives:
                        odrives.append(odrv)
                        print(f"✅ ODrive trouvé : SN {odrv.serial_number}")
                except:
                    break
    except Exception as e:
        print(f"❌ Erreur lors de la recherche: {e}")
    
    if not odrives:
        print("\n❌ AUCUN ODrive trouvé !")
        print("\nVérifiez :")
        print("  1. Les ODrives sont allumés (LEDs allumées)")
        print("  2. Les câbles USB sont bien connectés")
        print("  3. Vous avez les permissions (ajoutez votre user au groupe dialout)")
        print("     sudo usermod -a -G dialout $USER")
        return
    
    print(f"\n{'='*60}")
    print(f"NOMBRE D'ODRIVES TROUVÉS: {len(odrives)}")
    print(f"{'='*60}")
    
    # Vérifier chaque ODrive
    results = []
    for i, odrv in enumerate(odrives):
        result = check_odrive_can(odrv, i)
        results.append((i, odrv.serial_number, result))
    
    # Résumé final
    print(f"\n{'='*60}")
    print("RÉSUMÉ")
    print(f"{'='*60}")
    
    configured_count = sum(1 for _, _, ok in results if ok)
    print(f"\nODrives correctement configurés: {configured_count}/{len(odrives)}")
    
    print("\nListe des ODrives:")
    for i, sn, ok in results:
        status = "✅ OK" if ok else "❌ À CONFIGURER"
        print(f"  ODrive #{i} (SN: {sn}): {status}")
    
    if configured_count < len(odrives):
        print("\n⚠️  Certains ODrives nécessitent une configuration !")
        print("Utilisez le script 'configure_odrive_can.py' pour les configurer.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrompu par l'utilisateur")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ ERREUR FATALE: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)







