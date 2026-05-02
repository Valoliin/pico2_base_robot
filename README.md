# Contrôleur Robot - Raspberry Pi Pico 2 & ROS 2 (Micro-ROS)

Une bibliothèque C modulaire et robuste pour piloter les moteurs pas-à-pas en boucle fermée **MKS SERVO42E** via une liaison RS485 (module MAX3485) depuis une Raspberry Pi Pico 2 et trois paa5100je pour génerer une odométrie.
Le système communique désormais de manière autonome et asynchrone avec un Raspberry Pi 5 via **Micro-ROS**, en utilisant les deux cœurs du Pico 2 pour garantir des performances temps réel.

---

## 🚀 Interface ROS 2 (Micro-ROS)

Le Pico 2 exécute un nœud Micro-ROS nommé `pico_robot_node`.

### 1. Démarrer l'Agent sur le Raspberry Pi 5
Le Pico communique en série (UART0) sur les broches TX: 12 et RX: 13.
Pour établir la communication, lancez l'agent Micro-ROS sur le Pi 5 avec la commande suivante :
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 115200
```
*Note : Le Pico 2 se reconnectera automatiquement en cas de perte de l'agent.*

### 2. Les Topics disponibles

| Nom du Topic        | Type de Message             | Sens  | Description                                                        |
| ------------------- | --------------------------- | ----- | ------------------------------------------------------------------ |
| `/pico/cmd_simple`  | `geometry_msgs/msg/Point32` | 📥 Sub | Consigne de vitesse (RPM). `x` = Avant, `y` = Gauche, `z` = Droit. |
| `/pico/odom_simple` | `geometry_msgs/msg/Point32` | 📤 Pub | Odométrie (à 50 Hz). `x` = Pos X, `y` = Pos Y, `z` = Angle Theta.  |
| `/pico/reset_odom`  | `std_msgs/msg/Empty`        | 📥 Sub | Remet l'odométrie (X, Y, Theta) à zéro.                            |

### 3. Sécurité (Timeouts) ⚠️
Le robot possède un chien de garde (Watchdog) matériel strict pour éviter les accidents :
* **0.5 seconde sans message** : Freinage actif (Vitesse = 0, couple maintenu).
* **10 secondes sans message** : Coupure de l'énergie (Roue libre pour économiser la batterie).
👉 Il est donc **impératif de publier la consigne en continu** (par exemple à 10 Hz) pour faire rouler le robot.

### 4. Commandes de Test depuis le terminal (Pi 5)

**Faire tourner les moteurs (Publication à 10 Hz grâce à `-r 10`) :**
```bash
ros2 topic pub -r 10 /pico/cmd_simple geometry_msgs/msg/Point32 "{x: 50.0, y: -50.0, z: 0.0}"
```
*Pour stopper le robot, faites `Ctrl+C`. Le timeout de sécurité coupera les moteurs 500ms plus tard.*

**Lire la position calculée par les capteurs optiques :**
```bash
ros2 topic echo /pico/odom_simple
```

**Remettre l'odométrie à zéro :**
```bash
ros2 topic pub --once /pico/reset_odom std_msgs/msg/Empty 
```

---
### 5. Codes Couleur de la LED (Statut)
La LED RGB intégrée au Pico 2 indique l'état en temps réel du robot :
* 🔴 **Rouge** : Démarrage et initialisation.
* 🟪 **Violet** : Moteurs RS485 configurés.
* 🟨 **Jaune** : En attente de l'Agent Micro-ROS (Recherche de la Pi 5).
* 🟩 **Vert** : Connecté à ROS 2, prêt à recevoir des ordres !
* 🟦 **Bleu (Flash)** : Réception et exécution d'une commande de vitesse en cours.

---

## 🎯 Calibration de l'Odométrie (Capteurs PAA5100)

Pour que l'odométrie soit précise, il est crucial de calibrer deux types de paramètres :
1.  **Les constantes de conversion (`C1`, `C2`, `C3`)** : Elles transforment les "ticks" bruts de chaque capteur en mètres.
2.  **Le rayon du robot (`R_ROBOT`)** : Il est utilisé pour calculer la rotation à partir des déplacements des capteurs.

La calibration se fait en deux étapes principales.

### Étape 1 : Calibration des Constantes de Conversion (`C1`, `C2`, `C3`)

Cette étape utilise le mode de diagnostic pour obtenir les lectures brutes des capteurs.

1.  **Activation du mode diagnostic :**
    *   Dans le fichier `src/robot.cpp`, modifiez la constante pour activer le mode de calibration :
        ```bash
        #define CALIBRATION_MODE 1
        ```
    *   Compilez et flashez le code sur le Pico.

2.  **Mesure des ticks :**
    *   Lancez l'agent micro-ROS sur la Pi 5.
    *   Placez le robot sur une surface texturée, le long d'une règle.
    *   Écoutez les topics des capteurs pour voir les ticks bruts s'accumuler :
        ```bash
        ros2 topic echo /pico/capteur1
        ros2 topic echo /pico/capteur2
        ros2 topic echo /pico/capteur3
        ```
    *   Remettez les compteurs à zéro :
        ```bash
        ros2 topic pub --once /pico/reset_odom std_msgs/msg/Empty
        ```
    *   **Poussez le robot bien droit sur une distance connue**, par exemple **500 mm (0.5 m)**. Essayez de le pousser dans une direction qui correspond à l'axe `y` (tangentiel) des capteurs.
    *   Notez la valeur finale des ticks accumulés pour chaque capteur (par exemple, la valeur `y` du message `Point32`). Soit `ticks_capteur1`, `ticks_capteur2`, `ticks_capteur3`.

3.  **Calcul et mise à jour des constantes :**
    *   Calculez les constantes de conversion (mètres par tick) avec la formule :
        `C_nouveau = distance_reelle_en_metres / ticks_lus`
    *   Par exemple :
        `C1 = 0.5 / ticks_capteur1`
        `C2 = 0.5 / ticks_capteur2`
        `C3 = 0.5 / ticks_capteur3`
    *   Ouvrez le fichier `src/odom.cpp` et mettez à jour les valeurs des constantes `C1`, `C2`, `C3` avec les résultats de vos calculs.

### Étape 1.5 (Optionnel) : Vérification des constantes et de l'orientation

Ce mode permet de vérifier que les constantes `C` sont correctes et que les capteurs sont bien orientés.

1.  **Activation du mode de vérification :**
    *   Dans `src/robot.cpp`, passez au mode 2 : `#define CALIBRATION_MODE 2`
    *   Compilez et flashez le code.

2.  **Test de déplacement :**
    *   Poussez le robot en ligne droite sur une distance connue (ex: 50 cm).
    *   Écoutez les topics des capteurs : `ros2 topic echo /pico/capteur2`.
    *   La valeur `y` (ou `x` selon l'orientation) devrait correspondre à la distance parcourue en mètres (ex: `0.5`).
    *   Si une valeur est négative alors qu'elle devrait être positive, mettez le flag `invert_dx` ou `invert_dy` correspondant à `true` dans `src/odom.cpp` et recompilez.
    *   Ce mode est très utile pour s'assurer que tous les capteurs "voient" le mouvement dans le bon sens avant de passer à l'odométrie complète.

### Étape 2 : Calibration de la Rotation (`R_ROBOT`)

Maintenant que les capteurs mesurent correctement les distances, nous pouvons calibrer la rotation.

1.  **Retour au mode normal :**
    *   Dans `src/robot.cpp`, désactivez le mode de calibration :
        ```cpp
        #define CALIBRATION_MODE 0
        ```
    *   Recompilez et flashez le code.

2.  **Mesure de la rotation :**
    *   Écoutez le topic d'odométrie final :
        ```bash
        ros2 topic echo /pico/odom_simple
        ```
    *   Remettez l'odométrie à zéro :
        ```bash
        ros2 topic pub --once /pico/reset_odom std_msgs/msg/Empty
        ```
    *   **Faites pivoter le robot sur lui-même d'un nombre de tours connu**, par exemple 10 tours complets.
    *   Notez l'angle final `z` (theta) affiché par le topic `/pico/odom_simple`. C'est votre `Angle_Mesuré`.

3.  **Calcul et mise à jour du rayon :**
    *   Calculez l'angle théorique que vous auriez dû obtenir. Pour 10 tours : `Angle_Théorique = 10 * 2 * PI` (environ `62.83` radians).
    *   Ajustez la valeur de `R_ROBOT` dans `src/odom.cpp` avec la formule :
        `R_ROBOT_nouveau = R_ROBOT_actuel * (Angle_Mesuré / Angle_Théorique)`
    *   Si la calibration n'est pas parfaite, vous pouvez recompiler et répéter l'étape 2 pour affiner la valeur.

Une fois ces deux étapes terminées, votre odométrie devrait être bien calibrée !

---

## 📌 Fonctionnalités bas niveau (API C MKS SERVO)

Cette bibliothèque décompose la logique de communication pour offrir un contrôle total sur l'assemblage et l'envoi des trames :
* **Vitesse et Direction** (Commande `F6`) avec gestion de l'accélération.
* **Activation / Désactivation du moteur** (Commande `F3`) pour remplacer le fil physique "Enable".
* **Arrêt contrôlé** (Commande `F6` avec vitesse à 0) pour une décélération douce.
* **Arrêt d'urgence** (Commande `F7`) pour couper immédiatement le mouvement.
* **Lecture de l'acquittement** (`ACK`) avec timeout pour valider la bonne réception par le moteur.

---

## 🛠️ Pense-bête d'utilisation (Cheat Sheet)

Les trames MKS n'ont pas toutes la même longueur selon la commande envoyée. **Attention à bien ajuster les tailles** lors du calcul du Checksum (CRC) et de l'envoi.

On initialise toujours un buffer (ex: `uint8_t ma_trame[7];`) dans le `main` avant d'assembler les blocs.

### 1. Piloter la Vitesse (7 octets)
Paramètres : Adresse, Direction (1/0), Vitesse (0-3000 RPM), Accélération (0-255).
```c
mks_set_speed(ma_trame, 1, 1, 30, 16);       // Prépare les 6 premiers octets
ma_trame[6] = mks_get_checksum(ma_trame, 6); // Calcule le CRC sur 6 octets, index 6
mks_send(ma_trame, 7);                       // Envoie la trame totale de 7 octets
```

### 2. Arrêt Contrôlé / Doux (7 octets)
Paramètres : Adresse, Accélération (0 = immédiat, >0 = rampe de décélération).
```c
mks_set_stop(ma_trame, 1, 16);               // Décélération douce avec acc=16
ma_trame[6] = mks_get_checksum(ma_trame, 6); // Calcule le CRC sur 6 octets, index 6
mks_send(ma_trame, 7);                       // Envoie la trame totale de 7 octets
```

### 3. Activer / Désactiver le Moteur (5 octets)
Permet de libérer ou bloquer l'axe (remplace la broche Enable matérielle).
Paramètres : Adresse, État (1 = Activé, 0 = Désactivé/Roue libre).
```c
mks_set_enable(ma_trame, 1, 1);              // 1 = Enable
ma_trame[4] = mks_get_checksum(ma_trame, 4); // Calcule le CRC sur 4 octets, index 4
mks_send(ma_trame, 5);                       // Envoie la trame totale de 5 octets
```

### 4. Arrêt d'Urgence (4 octets)
Coupe instantanément la rotation. ⚠️ *Déconseillé si la vitesse est supérieure à 1000 RPM pour des raisons mécaniques.*
```c
mks_set_emergency_stop(ma_trame, 1);         // Prépare les 3 premiers octets
ma_trame[3] = mks_get_checksum(ma_trame, 3); // Calcule le CRC sur 3 octets, index 3
mks_send(ma_trame, 4);                       // Envoie la trame totale de 4 octets
```

### 5. Calibration de l'encodeur (5 octets)
Lance la procédure d'auto-calibration de l'encodeur magnétique pour le mode Closed-loop. 
⚠️ **L'axe du moteur doit être libre de toute contrainte mécanique (pignon/courroie débranchés) pendant l'opération.**
Paramètre : Adresse.
```c
mks_set_calibrate(ma_trame, 1);              // Prépare les 4 premiers octets
ma_trame[4] = mks_get_checksum(ma_trame, 4); // Calcule le CRC sur 4 octets, index 4
mks_send(ma_trame, 5);                       // Envoie la trame totale de 5 octets

// Attente du statut avec une fonction dédiée (timeout très long de 10s)
uint8_t status = mks_read_calib_status(); 
// status : 0 = En cours, 1 = Succès, 2 = Échec (bloqué)
```
### 6. Commande Groupée Multi-Moteurs (Long Data Package - 52 octets)

Permet d'envoyer des commandes simultanées jusqu'à 5 moteurs. La trame doit obligatoirement faire 52 octets, même si moins de 5 moteurs sont utilisés (les emplacements vides restent à zéro).
⚠️  **Note : Les moteurs ne renvoient aucun accusé de réception (ACK) pour cette commande. **

Exemple pour 3 moteurs :
```c
uint8_t trame_groupee[52];
uint8_t cmd_m1[7], cmd_m2[7], cmd_m3[7];

// 1. Initialise la trame avec des zéros et l'en-tête FC
mks_init_long_packet(trame_groupee);

// 2. Prépare les commandes individuelles (sans calculer leur CRC)
mks_set_speed(cmd_m1, 1, 1, 30, 16); // Moteur 1
mks_set_speed(cmd_m2, 2, 0, 30, 16); // Moteur 2
mks_set_speed(cmd_m3, 3, 1, 15, 10); // Moteur 3

// 3. Insère les commandes dans les emplacements (slots 0 à 4 possibles)
mks_add_to_long_packet(trame_groupee, 0, cmd_m1, 6);
mks_add_to_long_packet(trame_groupee, 1, cmd_m2, 6);
mks_add_to_long_packet(trame_groupee, 2, cmd_m3, 6);

// 4. Calcul du CRC global (sur 51 octets) et envoi
trame_groupee[51] = mks_get_checksum(trame_groupee, 51);
mks_send(trame_groupee, 52); 
// Pas de mks_read_ack() !
```