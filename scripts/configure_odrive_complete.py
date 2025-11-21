#!/usr/bin/env python3
"""
Script de configuration complète d'un ODrive v3.6 pour OpenDog v3
Configure moteur + encodeur + calibration + startup automatique

Usage:
    python3 configure_odrive_complete.py [serial_number]
    
Si serial_number n'est pas fourni, configure le premier ODrive trouvé.
"""

import odrive
from odrive.enums import *
import sys
import time

def wait_for_state(axis, target_state, timeout=30):
    """Attend que l'axe atteigne un état donné"""
    start_time = time.time()
    while axis.current_state != target_state:
        if time.time() - start_time > timeout:
            print(f"❌ TIMEOUT: L'axe n'a pas atteint l'état {target_state}")
            return False
        time.sleep(0.1)
    return True

def dump_errors(odrv):
    """Affiche toutes les erreurs de l'ODrive"""
    print("\n=== ERREURS ===")
    print(f"System: {odrv.error}")
    print(f"Axis0: {odrv.axis0.error}")
    print(f"  Motor: {odrv.axis0.motor.error}")
    print(f"  Encoder: {odrv.axis0.encoder.error}")
    print(f"  Controller: {odrv.axis0.controller.error}")
    print(f"Axis1: {odrv.axis1.error}")
    print(f"  Motor: {odrv.axis1.motor.error}")
    print(f"  Encoder: {odrv.axis1.encoder.error}")
    print(f"  Controller: {odrv.axis1.controller.error}")
    print()

def configure_axis(odrv, axis_num):
    """Configure un axe complet (moteur + encodeur + calibration)"""
    
    axis = getattr(odrv, f"axis{axis_num}")
    
    print(f"\n{'='*60}")
    print(f"CONFIGURATION AXIS {axis_num}")
    print(f"{'='*60}\n")
    
    # 1. Configuration du moteur
    print(f"1️⃣  Configuration du moteur...")
    axis.motor.config.pole_pairs = 20
    axis.motor.config.torque_constant = 8.27 / 100
    axis.motor.config.motor_type = 0  # MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.current_lim = 10.0  # Limite de courant pour fonctionnement
    axis.motor.config.current_lim_margin = 5.0
    axis.motor.config.calibration_current = 5.0  # Courant de calibration réduit à 5A
    axis.motor.config.resistance_calib_max_voltage = 2.0  # Tension réduite
    axis.motor.config.requested_current_range = 15.0  # Plage de courant réduite
    print("   ✅ Moteur configuré (calibration à 5A, limite opérationnelle à 10A)")
    
    # 2. Configuration de l'encodeur SPI
    print(f"2️⃣  Configuration de l'encodeur SPI...")
    axis.encoder.config.mode = 257  # MODE_SPI_ABS_AMS
    axis.encoder.config.cpr = 16384
    axis.encoder.config.calib_scan_distance = 150.0
    
    if axis_num == 0:
        axis.encoder.config.abs_spi_cs_gpio_pin = 5
    else:
        axis.encoder.config.abs_spi_cs_gpio_pin = 4
    
    print("   ✅ Encodeur SPI configuré")
    
    # 3. Configuration du contrôleur
    print(f"3️⃣  Configuration du contrôleur...")
    axis.controller.config.control_mode = 3  # CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = 5    # INPUT_MODE_TRAP_TRAJ
    axis.controller.config.vel_limit = 50.0
    axis.controller.config.pos_gain = 20.0
    axis.controller.config.vel_gain = 0.16
    axis.controller.config.vel_integrator_gain = 0.32
    print("   ✅ Contrôleur configuré")
    
    # 4. Effacer les erreurs
    print(f"4️⃣  Effacement des erreurs...")
    odrv.clear_errors()
    time.sleep(0.5)
    
    # 5. Calibration du moteur
    print(f"5️⃣  Calibration du moteur (peut prendre 10-15 secondes)...")
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    if not wait_for_state(axis, AXIS_STATE_IDLE, timeout=30):
        print("   ❌ Échec de la calibration moteur")
        dump_errors(odrv)
        return False
    
    if axis.motor.error != 0:
        print("   ❌ Erreur moteur détectée")
        dump_errors(odrv)
        return False
    
    print(f"   ✅ Moteur calibré")
    print(f"      Phase resistance: {axis.motor.config.phase_resistance:.6f} Ω")
    print(f"      Phase inductance: {axis.motor.config.phase_inductance:.9f} H")
    
    # Marquer le moteur comme pré-calibré
    axis.motor.config.pre_calibrated = True
    
    # 6. Calibration de l'encodeur (offset)
    print(f"6️⃣  Calibration de l'encodeur offset (peut prendre 10-15 secondes)...")
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_state(axis, AXIS_STATE_IDLE, timeout=30):
        print("   ❌ Échec de la calibration encodeur")
        dump_errors(odrv)
        return False
    
    if axis.encoder.error != 0:
        print("   ❌ Erreur encodeur détectée")
        dump_errors(odrv)
        return False
    
    print(f"   ✅ Encodeur calibré")
    
    # Marquer l'encodeur comme pré-calibré
    axis.encoder.config.pre_calibrated = True
    
    # 7. Configuration du démarrage automatique
    print(f"7️⃣  Configuration du démarrage automatique...")
    axis.config.startup_encoder_index_search = False
    axis.config.startup_encoder_offset_calibration = False
    axis.config.startup_motor_calibration = False
    axis.config.startup_closed_loop_control = True
    print("   ✅ Démarrage automatique en CLOSED_LOOP activé")
    
    # 8. Test rapide en CLOSED_LOOP
    print(f"8️⃣  Test en CLOSED_LOOP...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)
    
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("   ❌ Échec du passage en CLOSED_LOOP")
        dump_errors(odrv)
        return False
    
    print(f"   ✅ CLOSED_LOOP OK")
    print(f"      Position: {axis.encoder.pos_estimate:.3f} tours")
    print(f"      Vitesse: {axis.encoder.vel_estimate:.3f} tours/s")
    
    # Retour en IDLE
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    
    print(f"\n✅ AXIS {axis_num} CONFIGURÉ AVEC SUCCÈS !\n")
    return True

def main():
    print("="*60)
    print("CONFIGURATION COMPLÈTE ODRIVE v3.6 POUR OPENDOG v3")
    print("="*60)
    
    # Connexion à l'ODrive
    print("\n🔍 Recherche d'un ODrive...")
    
    if len(sys.argv) > 1:
        serial_number = sys.argv[1]
        print(f"   Connexion au serial: {serial_number}")
        try:
            odrv = odrive.find_any(serial_number=serial_number)
        except:
            print(f"❌ Impossible de trouver l'ODrive avec serial {serial_number}")
            return 1
    else:
        print("   Connexion au premier ODrive trouvé...")
        try:
            odrv = odrive.find_any()
        except:
            print("❌ Aucun ODrive trouvé !")
            print("\n💡 Vérifiez que l'ODrive est connecté en USB")
            return 1
    
    print(f"✅ ODrive connecté !")
    print(f"   Serial: {odrv.serial_number}")
    print(f"   Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"   Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
    
    # Configuration de base
    print("\n📋 Configuration de base...")
    odrv.config.enable_brake_resistor = True
    odrv.config.brake_resistance = 2.0
    odrv.config.dc_bus_overvoltage_trip_level = 56.0
    odrv.config.dc_max_positive_current = 20.0
    odrv.config.dc_max_negative_current = -3.0
    print("   ✅ Configuration de base OK")
    
    # Demander quels axes configurer
    print("\n❓ Quels axes voulez-vous configurer ?")
    print("   1. Axis 0 seulement")
    print("   2. Axis 1 seulement")
    print("   3. Les deux axes")
    
    choice = input("\nVotre choix (1/2/3) [3]: ").strip()
    if not choice:
        choice = "3"
    
    axes_to_configure = []
    if choice == "1":
        axes_to_configure = [0]
    elif choice == "2":
        axes_to_configure = [1]
    else:
        axes_to_configure = [0, 1]
    
    # Configuration des axes
    success = True
    for axis_num in axes_to_configure:
        if not configure_axis(odrv, axis_num):
            success = False
            print(f"\n❌ Échec de la configuration de l'Axis {axis_num}")
            break
    
    if not success:
        print("\n❌ CONFIGURATION ÉCHOUÉE")
        print("\n💡 Vérifiez :")
        print("   - Les connexions moteur (3 phases)")
        print("   - Les connexions encodeur (SPI)")
        print("   - L'alimentation (> 24V)")
        return 1
    
    # Sauvegarde de la configuration
    print("\n💾 Sauvegarde de la configuration...")
    try:
        odrv.save_configuration()
        print("   ✅ Configuration sauvegardée")
    except:
        print("   ❌ Échec de la sauvegarde")
        return 1
    
    print("\n" + "="*60)
    print("✅ CONFIGURATION TERMINÉE AVEC SUCCÈS !")
    print("="*60)
    print("\n📋 Résumé :")
    for axis_num in axes_to_configure:
        print(f"   ✅ Axis {axis_num} : Moteur calibré, Encodeur calibré, Startup CLOSED_LOOP activé")
    
    print("\n🔄 REDÉMARRAGE RECOMMANDÉ")
    print("   Après le redémarrage, l'ODrive démarrera automatiquement en CLOSED_LOOP")
    
    reboot = input("\n❓ Redémarrer l'ODrive maintenant ? (o/n) [o]: ").strip().lower()
    if not reboot or reboot == "o":
        print("\n🔄 Redémarrage de l'ODrive...")
        try:
            odrv.reboot()
            print("   ✅ ODrive redémarré")
        except:
            print("   ⚠️  Déconnexion normale pendant le redémarrage")
    
    print("\n✅ TERMINÉ !")
    return 0

if __name__ == "__main__":
    sys.exit(main())

