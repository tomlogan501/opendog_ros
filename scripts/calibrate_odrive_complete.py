#!/usr/bin/env python3
"""
Script de calibration complète ODrive v3.6 avec encodeurs SPI AMS AS5047P
Suit la procédure officielle OpenDog
"""

import odrive
from odrive.enums import *
import time
import sys

def wait_for_calibration(axis, timeout=30):
    """Attend la fin de la calibration"""
    print("  Calibration en cours", end="", flush=True)
    start_time = time.time()
    
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print(" ❌ TIMEOUT")
            return False
        print(".", end="", flush=True)
        time.sleep(0.5)
    
    print(" ✅ Terminé")
    return True

def check_errors(odrv, axis_name):
    """Vérifie les erreurs d'un axe"""
    axis = getattr(odrv, axis_name)
    
    if axis.error != 0:
        print(f"  ❌ {axis_name} error: 0x{axis.error:X}")
        dump_errors(odrv)
        return False
    
    if axis.motor.error != 0:
        print(f"  ❌ {axis_name} motor error: 0x{axis.motor.error:X}")
        return False
    
    if axis.encoder.error != 0:
        print(f"  ❌ {axis_name} encoder error: 0x{axis.encoder.error:X}")
        return False
    
    print(f"  ✅ {axis_name} OK")
    return True

def main():
    print("=" * 60)
    print("CALIBRATION COMPLÈTE ODRIVE v3.6 - ENCODEURS SPI")
    print("=" * 60)
    
    # Connexion
    print("\n[1/8] Connexion à l'ODrive...")
    try:
        odrv0 = odrive.find_any()
        print(f"  ✅ Connecté: {odrv0.serial_number}")
    except Exception as e:
        print(f"  ❌ Erreur: {e}")
        sys.exit(1)
    
    # Vérification encodeurs
    print("\n[2/8] Vérification des encodeurs SPI...")
    shadow0_before = odrv0.axis0.encoder.shadow_count
    shadow1_before = odrv0.axis1.encoder.shadow_count
    print(f"  Axis0 shadow_count: {shadow0_before}")
    print(f"  Axis1 shadow_count: {shadow1_before}")
    
    if shadow0_before == 0 and shadow1_before == 0:
        print("  ❌ Les encodeurs ne lisent rien ! Vérifiez le câblage SPI.")
        sys.exit(1)
    
    print("  ✅ Les encodeurs fonctionnent")
    
    # Configuration de base (déjà faite normalement)
    print("\n[3/8] Vérification configuration de base...")
    print(f"  brake_resistance: {odrv0.config.brake_resistance}")
    print(f"  dc_max_positive_current: {odrv0.config.dc_max_positive_current}")
    print(f"  Axis0 current_lim: {odrv0.axis0.motor.config.current_lim}")
    print(f"  Axis1 current_lim: {odrv0.axis1.motor.config.current_lim}")
    
    # IMPORTANT: Augmenter le courant de calibration
    print("\n[4/8] Configuration courant de calibration...")
    odrv0.axis0.motor.config.calibration_current = 10.0
    odrv0.axis1.motor.config.calibration_current = 10.0
    print("  ✅ calibration_current = 10A (pour les 2 axes)")
    
    # Vérifier que le moteur est bien fixé
    print("\n⚠️  ATTENTION ⚠️")
    print("  Les moteurs DOIVENT être fixés solidement !")
    print("  Ils vont tourner pendant la calibration.")
    response = input("\n  Moteurs fixés ? (oui/non): ")
    if response.lower() not in ['oui', 'o', 'yes', 'y']:
        print("  ❌ Calibration annulée")
        sys.exit(0)
    
    # Clear errors
    print("\n[5/8] Nettoyage des erreurs...")
    odrv0.clear_errors()
    time.sleep(0.5)
    
    # Calibration MOTEUR uniquement (pas l'encodeur encore)
    print("\n[6/8] Calibration MOTEUR Axis0...")
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis0, timeout=30):
        print("  ❌ Échec calibration moteur Axis0")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis0"):
        print("  ❌ Erreur après calibration moteur Axis0")
        sys.exit(1)
    
    # Sauvegarder les paramètres moteur
    print("  💾 Sauvegarde phase_resistance et phase_inductance...")
    print(f"     phase_resistance: {odrv0.axis0.motor.config.phase_resistance:.6f}")
    print(f"     phase_inductance: {odrv0.axis0.motor.config.phase_inductance:.9f}")
    odrv0.axis0.motor.config.pre_calibrated = True
    
    print("\n[6/8] Calibration MOTEUR Axis1...")
    odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis1, timeout=30):
        print("  ❌ Échec calibration moteur Axis1")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis1"):
        print("  ❌ Erreur après calibration moteur Axis1")
        sys.exit(1)
    
    print("  💾 Sauvegarde phase_resistance et phase_inductance...")
    print(f"     phase_resistance: {odrv0.axis1.motor.config.phase_resistance:.6f}")
    print(f"     phase_inductance: {odrv0.axis1.motor.config.phase_inductance:.9f}")
    odrv0.axis1.motor.config.pre_calibrated = True
    
    # Calibration ENCODEUR offset
    print("\n[7/8] Calibration ENCODEUR OFFSET Axis0...")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis0, timeout=30):
        print("  ❌ Échec calibration encodeur Axis0")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis0"):
        print("  ❌ Erreur après calibration encodeur Axis0")
        sys.exit(1)
    
    print(f"  💾 Offset sauvegardé: {odrv0.axis0.encoder.config.offset}")
    odrv0.axis0.encoder.config.pre_calibrated = True
    
    print("\n[7/8] Calibration ENCODEUR OFFSET Axis1...")
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis1, timeout=30):
        print("  ❌ Échec calibration encodeur Axis1")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis1"):
        print("  ❌ Erreur après calibration encodeur Axis1")
        sys.exit(1)
    
    print(f"  💾 Offset sauvegardé: {odrv0.axis1.encoder.config.offset}")
    odrv0.axis1.encoder.config.pre_calibrated = True
    
    # Configuration startup
    print("\n[8/8] Configuration startup closed loop...")
    odrv0.axis0.config.startup_motor_calibration = False
    odrv0.axis0.config.startup_encoder_index_search = False
    odrv0.axis0.config.startup_encoder_offset_calibration = False
    odrv0.axis0.config.startup_closed_loop_control = True
    
    odrv0.axis1.config.startup_motor_calibration = False
    odrv0.axis1.config.startup_encoder_index_search = False
    odrv0.axis1.config.startup_encoder_offset_calibration = False
    odrv0.axis1.config.startup_closed_loop_control = True
    
    print("  ✅ Startup configuré")
    
    # Sauvegarde finale
    print("\n💾 Sauvegarde configuration...")
    odrv0.save_configuration()
    print("  ✅ Configuration sauvegardée")
    
    print("\n" + "=" * 60)
    print("✅✅✅ CALIBRATION COMPLÈTE RÉUSSIE ! ✅✅✅")
    print("=" * 60)
    print("\n🎉 Les 2 axes sont calibrés et prêts pour CLOSED_LOOP_CONTROL")
    print("\n⚠️  L'ODrive va redémarrer...")
    
    odrv0.reboot()
    
    print("\n📋 PROCHAINES ÉTAPES :")
    print("  1. Attendez 5 secondes que l'ODrive redémarre")
    print("  2. Reconnectez-vous avec odrivetool")
    print("  3. Les axes devraient être en CLOSED_LOOP_CONTROL automatiquement")
    print("  4. Testez avec: odrv0.axis0.controller.input_pos = 1")
    print("  5. Lancez le hardware layer ROS2 !")

if __name__ == "__main__":
    main()





