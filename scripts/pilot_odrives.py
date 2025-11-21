import odrive
from odrive.enums import *
import time
import math

# =======================
# Constantes globales
# =======================
CURRENT_LIMIT_HIGH = 40.0      # A → pour les moteurs à fort courant (ex-épaule)
CURRENT_LIT_LOW = 2.5          # A → pour les autres moteurs
TEST_INCREMENT = 0.2           # rad → petit test de déplacement
LOOP_PERIOD = 0.02             # s → fréquence de boucle (50 Hz)
SIN_FREQ = 2.0                 # Hz → fréquence du mouvement sinusoïdal
SIN_AMP_LEG = 0.4              # rad → amplitude pour jambes
SIN_AMP_HIP = 0.2              # rad → amplitude pour hanches

# Définition des hanches et offsets
HIPS_AXES = [0x2, 0x3, 0x8, 0x9]
HIPS_OFFSETS = {0x2: 0.2, 0x3: 0.1, 0x8: 0.15, 0x9: 0.15}

# Offsets spécifiques des épaules (pour fixation en continu)
SHOULDER_OFFSETS = {
    "AVANT_GAUCHE": 0.2,
    "AVANT_DROITE": -0.2,
    "ARRIERE_GAUCHE": 0.2,
    "ARRIERE_DROITE": -0.2
}


def main():
    print("Connexion aux ODrives...")

    # Numéros de série de tes 6 ODrives
    serials = [
        "335536633539",  # autre
        "335836543539",  # autre
        "3359366C3539",  # fort courant avant gauche/droite
        "3359366F3539",  # autre
        "335436563539",  # autre
        "3673385F3030"   # fort courant arrière gauche/droite
    ]

    odrives = []
    for sn in serials:
        print(f"Connexion à l'ODrive SN {sn} ...")
        odrv = odrive.find_any(serial_number=sn)
        odrives.append(odrv)
        print(f"  -> Connecté : {odrv.serial_number}")

    print("\nTous les ODrives sont connectés.")

    # --- Création des tables d'axes ---
    AxisTab = []  # tous les axes (odrive, axis_id)
    for odrv in odrives:
        AxisTab.append((odrv, 0))  # axis0
        AxisTab.append((odrv, 1))  # axis1

    # --- Moteurs à courant fort (anciennement épaules) ---
    high_current_indices = [2, 5]  # indices des ODrives à fort courant
    AxisTabCourantFort = []
    for i, odrv in enumerate(odrives):
        if i in high_current_indices:
            AxisTabCourantFort.append((odrv, 0))
            AxisTabCourantFort.append((odrv, 1))

    # --- Fonction pour lire l'état d'un axe ---
    def print_axis_state(odrv, axis_id, axis_name=""):
        if axis_id == 0:
            axis = odrv.axis0
        else:
            axis = odrv.axis1

        print(f"\n{axis_name} | Carte {odrv.serial_number} Axis{axis_id}:")
        print(f"  -> Current State: {axis.current_state}")
        print(f"  -> Axis Error: {axis.error}")
        print(f"  -> Motor Error: {axis.motor.error}")
        print(f"  -> Encoder Error: {axis.encoder.error}")
        print(f"  -> Encoder Pos Estimate: {axis.encoder.pos_estimate:.3f} rad")
        print(f"  -> Encoder Vel Estimate: {axis.encoder.vel_estimate:.3f} rad/s")

    # --- Limites de courant ---
    print("\nConfiguration des courants :")
    for odrv, axis_id in AxisTab:
        is_high_current = any(odrv is hc_odrv for hc_odrv, _ in AxisTabCourantFort)
        
        if is_high_current:
            current_limit = CURRENT_LIMIT_HIGH
            print(f"  -> COURANT FORT | Carte {odrv.serial_number} Axis{axis_id} | Courant = {current_limit} A")
        else:
            current_limit = CURRENT_LIT_LOW
            print(f"  -> COURANT FAIBLE | Carte {odrv.serial_number} Axis{axis_id} | Courant = {current_limit} A")

        if axis_id == 0:
            odrv.axis0.motor.config.current_lim = current_limit
        else:
            odrv.axis1.motor.config.current_lim = current_limit

    # Passer en mode position (Closed Loop)
    for odrv in odrives:
        for axis in [odrv.axis0, odrv.axis1]:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    time.sleep(1)  # stabilisation

    # --- Lecture initiale des positions ---
    pos_init_map = {}
    for odrv in odrives:
        pos_init_map[(odrv, 0)] = odrv.axis0.encoder.pos_estimate
        pos_init_map[(odrv, 1)] = odrv.axis1.encoder.pos_estimate

    print("\n=== POSITIONS INITIALES DE TOUS LES AXES ===")
    for idx, ((odrv, axis_id), pos) in enumerate(pos_init_map.items()):
        if (odrv, axis_id) in AxisTabCourantFort:
            type_axe = "COURANT FORT"
        elif axis_id in HIPS_AXES:
            type_axe = "HANCHE"
        else:
            type_axe = "JAMBE"
        print(f"  {type_axe} | Carte {odrv.serial_number} Axis{axis_id} = {pos:.3f} rad")

    # --- Application des offsets spécifiques aux épaules ---
    shoulder_offsets_map = {
        (odrives[2], 0): pos_init_map[(odrives[2], 0)] + SHOULDER_OFFSETS["AVANT_GAUCHE"],
        (odrives[2], 1): pos_init_map[(odrives[2], 1)] + SHOULDER_OFFSETS["AVANT_DROITE"],
        (odrives[5], 0): pos_init_map[(odrives[5], 0)] + SHOULDER_OFFSETS["ARRIERE_GAUCHE"],
        (odrives[5], 1): pos_init_map[(odrives[5], 1)] + SHOULDER_OFFSETS["ARRIERE_DROITE"]
    }

    print("\nFixation initiale des épaules avec offsets...")
    for (odrv, axis_id), target_pos in shoulder_offsets_map.items():
        if axis_id == 0:
            odrv.axis0.controller.input_pos = target_pos
        else:
            odrv.axis1.controller.input_pos = target_pos
        
        # Déterminer le nom de l'épaule pour l'affichage
        if odrv == odrives[2]:
            position = "AVANT"
            side = "GAUCHE" if axis_id == 0 else "DROITE"
        else:
            position = "ARRIÈRE"
            side = "GAUCHE" if axis_id == 0 else "DROITE"
        
        print(f"  -> ÉPAULE {position} {side} | Carte {odrv.serial_number} Axis{axis_id} = {target_pos:.3f} rad")

    # --- Boucle sinusoidale (jambes + hanches uniquement) ---
    t0 = time.monotonic()
    last_display_time = t0
    last_state_display_time = t0
    print("\nBoucle mouvement sinusoidal en cours (épaules fixes)... Ctrl+C pour arrêter")
    print("Surveillance des positions toutes les 3 secondes...")

    try:
        while True:
            t1 = time.monotonic()

            # Afficher périodiquement l'état des positions + courants
            if t1 - last_display_time > 3.0:  # Toutes les 3 secondes
                total_current = 0.0

                print("\n=== ÉTAT ACTUEL DES POSITIONS & COURANTS ===")
                print("COURANT FORT (épaules avec offsets):")
                for (odrv, axis_id) in shoulder_offsets_map:
                    if axis_id == 0:
                        current_pos = odrv.axis0.encoder.pos_estimate
                        m = odrv.axis0.motor
                    else:
                        current_pos = odrv.axis1.encoder.pos_estimate
                        m = odrv.axis1.motor

                    current_cur = (abs(m.current_meas_phA) + abs(m.current_meas_phB) + abs(m.current_meas_phC)) / 3.0
                    target_pos = shoulder_offsets_map[(odrv, axis_id)]
                    deviation = abs(current_pos - target_pos)
                    total_current += current_cur

                    # Déterminer le nom de l'épaule pour l'affichage
                    if odrv == odrives[2]:
                        position = "AVANT"
                        side = "GAUCHE" if axis_id == 0 else "DROITE"
                    else:
                        position = "ARRIÈRE"
                        side = "GAUCHE" if axis_id == 0 else "DROITE"
                    
                    print(f"  ÉPAULE {position} {side} | Carte {odrv.serial_number} Axis{axis_id}: "
                          f"{current_pos:.3f} rad (cible: {target_pos:.3f}, écart: {deviation:.3f}) "
                          f"| Courant: {current_cur:.2f} A")

                print("\nAUTRES ARTICULATIONS (en mouvement sinusoïdal):")
                for (odrv, axis_id) in AxisTab:
                    if (odrv, axis_id) not in shoulder_offsets_map:
                        if axis_id == 0:
                            current_pos = odrv.axis0.encoder.pos_estimate
                            m = odrv.axis0.motor
                        else:
                            current_pos = odrv.axis1.encoder.pos_estimate
                            m = odrv.axis1.motor

                        current_cur = (abs(m.current_meas_phA) + abs(m.current_meas_phB) + abs(m.current_meas_phC)) / 3.0
                        init_pos = pos_init_map[(odrv, axis_id)]
                        total_current += current_cur

                        if axis_id in HIPS_AXES:
                            type_axe = "HANCHE"
                        else:
                            type_axe = "JAMBE"
                            
                        print(f"  {type_axe} | Carte {odrv.serial_number} Axis{axis_id}: "
                              f"{current_pos:.3f} rad (init: {init_pos:.3f}) "
                              f"| Courant: {current_cur:.2f} A")

                print(f"\n>>> Consommation totale estimée = {total_current:.2f} A <<<")
                last_display_time = t1

            # Afficher périodiquement l'état des axes
            if t1 - last_state_display_time > 10.0:
                print("\n=== ÉTATS DES AXES ===")
                for odrv, axis_id in AxisTab:
                    if (odrv, axis_id) in shoulder_offsets_map:
                        if odrv == odrives[2]:
                            position = "AVANT"
                            side = "GAUCHE" if axis_id == 0 else "DROITE"
                            type_axe = f"ÉPAULE {position} {side}"
                        else:
                            position = "ARRIÈRE"
                            side = "GAUCHE" if axis_id == 0 else "DROITE"
                            type_axe = f"ÉPAULE {position} {side}"
                    elif axis_id in HIPS_AXES:
                        type_axe = "HANCHE"
                    else:
                        type_axe = "JAMBE"
                    print_axis_state(odrv, axis_id, type_axe)
                last_state_display_time = t1

            # Consignes sinus pour les hanches et jambes
            setpoint_leg = SIN_AMP_LEG * math.sin((t1 - t0) * SIN_FREQ)
            setpoint_hip = SIN_AMP_HIP * math.sin((t1 - t0) * SIN_FREQ)

            for (odrv, axis_id) in AxisTab:
                if (odrv, axis_id) in shoulder_offsets_map:
                    # Maintenir les épaules fixes avec leurs offsets
                    sp = shoulder_offsets_map[(odrv, axis_id)]
                elif axis_id in HIPS_AXES:
                    # Mouvement sinusoïdal pour les hanches
                    sp = setpoint_hip + HIPS_OFFSETS.get(axis_id, 0.0)
                else:
                    # Mouvement sinusoïdal pour les jambes
                    sp = setpoint_leg

                if axis_id == 0:
                    odrv.axis0.controller.input_pos = sp
                else:
                    odrv.axis1.controller.input_pos = sp

            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur.")


if __name__ == "__main__":
    main()