/**
 * ============================================================================
 *   🤖 CERVEAU DU ROBOT (BASE ROULANTE) - RASPBERRY PI PICO 2
 * ============================================================================
 * Ce fichier est le cœur du robot. Il utilise les deux processeurs (coeurs) :
 * - Coeur 0 : Discute avec l'ordinateur principal (Raspberry Pi 5) et gère la sécurité.
 * - Coeur 1 : Lit les capteurs optiques sous le robot pour calculer sa position.
 */
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "lib/paa5100/pmw3901.hpp"
#include "lib/mks_servo/mks_servo.h"
#include "odom.hpp"

// --- INCLUDE MICRO-ROS ---
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/point32.h>
#include <std_msgs/msg/empty.h>

extern "C"
{
#include "pico_uart_transports.h"
}

using namespace pimoroni;

// ==========================================
// CONFIGURATION MATÉRIELLE
// ==========================================
// ============================================================================
// 1. CONFIGURATION MATÉRIELLE (LES CAPTEURS)
// ============================================================================
// Les broches (pins) pour communiquer avec les capteurs optiques en SPI
const uint PIN_SCK = 18, PIN_MOSI = 19, PIN_MISO = 20;
// Création de nos 3 capteurs "souris" qui regardent le sol
PAA5100 capteur2(spi0, 21, PIN_SCK, PIN_MOSI, PIN_MISO, 22);
PAA5100 capteur1(spi0, 17, PIN_SCK, PIN_MOSI, PIN_MISO, 16);
PAA5100 capteur3(spi0, 15, PIN_SCK, PIN_MOSI, PIN_MISO, 14);

// ============================================================================
// MODE DIAGNOSTIC UART
// ============================================================================
#define DEBUG_UART 0 // Mettre à 1 pour activer le test UART bloquant au démarrage

// ==========================================
// CONFIGURATION MODULAIRE
// ==========================================
// ============================================================================
// 2. CONFIGURATION DES MOTEURS<
// ============================================================================
// Les "noms" (adresses) de nos moteurs sur le câble RS485
uint8_t ID_MOT_AVANT = 2;
uint8_t ID_MOT_GAUCHE = 1;
uint8_t ID_MOT_DROIT = 3;

// Si on monte un moteur à l'envers mécaniquement, on met "true" ici pour corriger
bool inv_m1 = false;
bool inv_m2 = false;
bool inv_m3 = false;

// --- MUTEX : LES BÂTONS DE PAROLE ---
// Comme on a deux coeurs qui travaillent en même temps, ils ne doivent pas parler
// aux moteurs ou lire les mêmes variables en même temps.
// Celui qui veut faire une action doit d'abord prendre le Mutex (le bâton).
mutex_t mks_mutex;
mutex_t odom_mutex;

// --- GESTION DE LA SÉCURITÉ ---
bool inv_m1_en = false;
bool inv_m2_en = false; // Exemple : le moteur 2 est inversé
bool inv_m3_en = false;
// "volatile" prévient le processeur que l'autre coeur peut changer cette valeur à tout moment
volatile bool motors_enabled = false;
volatile bool motors_moving = false;

volatile uint32_t last_vel_time = 0;
const uint32_t STOP_TIMEOUT_MS = 500;       // Si la Pi5 ne parle plus pendant 0.5s -> On freine net !
const uint32_t DISABLE_TIMEOUT_MS = 10000;  // Si on attend depuis 10s -> On coupe l'énergie (roue libre)
const bool DISABLE_SAFETY_TIMEOUTS = false; // À mettre sur "true" uniquement pour tester au bureau
uint8_t vit_accel = 20;                     // Douceur de l'accélération (0=bourrin, 255=très lent)
const float MAX_SPEED_RPM = 100.0f;         // Vitesse maximale autorisée (Sécurité mécanique)

// ============================================================================
// 3. VARIABLES MICRO-ROS (LA COMMUNICATION AVEC LA PI 5)
// ============================================================================

// --- Déclaration des publishers en fonction du mode ---
#define CALIBRATION_MODE 0    // 0 = Odométrie | 1 = Ticks Bruts | 2 = Ticks en Mètres

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Point32 cmd_vel_msg;

#if CALIBRATION_MODE > 0 // Modes 1 et 2
rcl_publisher_t calib_publisher1;
rcl_publisher_t calib_publisher2;
rcl_publisher_t calib_publisher3;
geometry_msgs__msg__Point32 calib_msg1;
geometry_msgs__msg__Point32 calib_msg2;
geometry_msgs__msg__Point32 calib_msg3;
#else // Mode 0
rcl_publisher_t odom_simple_publisher;
geometry_msgs__msg__Point32 odom_simple_msg;
#endif

rcl_subscription_t reset_odom_subscriber;
std_msgs__msg__Empty reset_odom_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool micro_ros_init_successful = false;

// Ces variables magiques sont calculées en arrière-plan dans odom.cpp
extern float global_X, global_Y, global_Theta;

// ============================================================================
// MODE CALIBRATION DES CAPTEURS
// ============================================================================
#if CALIBRATION_MODE == 1
// Accumulateurs de ticks bruts pour chaque capteur
volatile float calib_X1 = 0.0f, calib_Y1 = 0.0f;
volatile float calib_X2 = 0.0f, calib_Y2 = 0.0f;
volatile float calib_X3 = 0.0f, calib_Y3 = 0.0f;
#elif CALIBRATION_MODE == 2
// Accumulateurs de déplacement en mètres pour chaque capteur
volatile float calib_mX1 = 0.0f, calib_mY1 = 0.0f;
volatile float calib_mX2 = 0.0f, calib_mY2 = 0.0f;
volatile float calib_mX3 = 0.0f, calib_mY3 = 0.0f;
#endif
// ============================================================================
// FONCTIONS UTILITAIRES
// ============================================================================
const uint LED_R = 28;
const uint LED_G = 27;
const uint LED_B = 26;

/**
 * Change la couleur de la LED sur la carte Pico pour savoir ce que fait le robot.
 * @param r (Rouge), @param g (Vert), @param b (Bleu). 1 = Allumé, 0 = Éteint.
 */
void set_rgb(bool r, bool g, bool b)
{
    gpio_put(LED_R, r);
    gpio_put(LED_G, g);
    gpio_put(LED_B, b);
}

/**
 * Détermine si on envoie un "1" ou un "0" au moteur pour l'activer,
 * en gérant le cas où certains moteurs sont câblés à l'envers.
 */
uint8_t get_mks_enable_state(uint8_t id, bool global_request)
{
    bool inverted = (id == 1 && inv_m1_en) || (id == 2 && inv_m2_en) || (id == 3 && inv_m3_en);
    return inverted ? !global_request : global_request;
}

/**
 * ============================================================================
 * L'OREILLE DU ROBOT : RÉCEPTION DES COMMANDES DE VITESSE
 * ============================================================================
 * Cette fonction s'active TOUTE SEULE à chaque fois que la Raspberry Pi 5
 * envoie un nouveau message sur le topic '/pico/cmd_simple'.
 * Elle convertit le message reçu en ordres électriques pour les moteurs.
 */
void cmd_vel_callback(const void *msgin)
{
    // On allume la LED en bleu pour montrer qu'on a reçu un ordre
    set_rgb(0, 0, 1);

    // On déballe le message reçu
    const geometry_msgs__msg__Point32 *msg_in = (const geometry_msgs__msg__Point32 *)msgin;

    // On range les vitesses (x, y, z) dans un tableau pour chaque moteur
    float speeds[3] = {msg_in->x, msg_in->y, msg_in->z};

    // On remet le chronomètre de sécurité à zéro
    last_vel_time = to_ms_since_boot(get_absolute_time());
    motors_moving = true;

    // On regarde si on nous demande d'avancer (vitesse != 0)
    bool moving = (speeds[0] != 0 || speeds[1] != 0 || speeds[2] != 0);

    // On prend le bâton de parole (Mutex) car on va utiliser le câble RS485
    mutex_enter_blocking(&mks_mutex); // On prend le bâton de parole pour les moteurs

    // Si le robot était endormi et qu'on lui demande de bouger, on réveille les moteurs
    if (moving && !motors_enabled)
    {
        uint8_t t[5];
        for (int i = 1; i <= 3; i++)
        {
            mks_set_enable(t, i, get_mks_enable_state(i, 1));
            t[4] = mks_get_checksum(t, 4);
            mks_send(t, 5);
            sleep_ms(20);
        }
        motors_enabled = true;

        // Pause de 20ms indispensable : il faut laisser le temps à la mécanique
        // du moteur de "s'armer" (faire le champ magnétique) avant d'envoyer la vitesse !
        sleep_ms(20);
    }

    // On prépare un "Colis Groupé" (Long Packet de 52 octets) pour envoyer
    // l'ordre aux 3 moteurs exactement en même temps.
    uint8_t trame[52];
    uint8_t c_data[3][7];
    mks_init_long_packet(trame);
    uint8_t ids[3] = {ID_MOT_AVANT, ID_MOT_GAUCHE, ID_MOT_DROIT};
    bool invs[3] = {inv_m1, inv_m2, inv_m3};

    for (int i = 0; i < 3; i++)
    {
        // Calcule le sens de rotation (1 ou 0)
        uint8_t dir = ((speeds[i] >= 0) ^ invs[i]) ? 1 : 0;

        float abs_speed = fabs(speeds[i]);

        // Sécurité : on empêche le robot d'aller trop vite
        if (abs_speed > MAX_SPEED_RPM)
            abs_speed = MAX_SPEED_RPM; // Écrêtage de sécurité

        // On range l'ordre du moteur 'i' dans notre colis groupé
        mks_set_speed(c_data[i], ids[i], dir, (uint16_t)abs_speed, vit_accel);
        mks_add_to_long_packet(trame, i, c_data[i], 6);
    }
    // Calcul de la signature de vérification (Checksum) et envoi physique
    trame[51] = mks_get_checksum(trame, 51);
    mks_send(trame, 52);

    // On a fini de parler avec les moteurs, on rend le bâton
    mutex_exit(&mks_mutex);
}

/**
 * ============================================================================
 * RÉINITIALISATION DE L'ODOMÉTRIE
 * ============================================================================
 * Cette fonction s'active à la réception d'un message sur '/pico/reset_odom'.
 */
void reset_odom_callback(const void *msgin)
{
    // On prend le Mutex pour ne pas modifier pendant que le Coeur 1 calcule
    mutex_enter_blocking(&odom_mutex);
#if CALIBRATION_MODE == 1
    calib_X1 = 0.0f; calib_Y1 = 0.0f; calib_X2 = 0.0f;
    calib_Y2 = 0.0f; calib_X3 = 0.0f; calib_Y3 = 0.0f;
#elif CALIBRATION_MODE == 2
    calib_mX1 = 0.0f; calib_mY1 = 0.0f;
    calib_mX2 = 0.0f; calib_mY2 = 0.0f;
    calib_mX3 = 0.0f; calib_mY3 = 0.0f;
#else
    global_X = 0.0f;
    global_Y = 0.0f;
    global_Theta = 0.0f;
#endif
    mutex_exit(&odom_mutex);

    set_rgb(0, 1, 1); // Clignotement Cyan pour confirmer la remise à zéro
}

/**
 * ============================================================================
 * BRANCHER LE TÉLÉPHONE : INITIALISATION MICRO-ROS
 * ============================================================================
 * Prépare tout le nécessaire pour discuter avec le "grand" réseau ROS 2.
 * On déclare nos "topics" d'écoute et de parole.
 */
bool create_entities()
{
    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK)
        return false;

    rc = rclc_node_init_default(&node, "pico_robot_node", "", &support);
    if (rc != RCL_RET_OK)
        return false;

    auto type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);

    rc = rclc_subscription_init_default(&cmd_vel_subscriber, &node, type_support, "pico/cmd_simple");
    if (rc != RCL_RET_OK)
        return false;

#if CALIBRATION_MODE > 0
    rc = rclc_publisher_init_default(&calib_publisher1, &node, type_support, "pico/capteur1");
    if (rc != RCL_RET_OK) return false;
    rc = rclc_publisher_init_default(&calib_publisher2, &node, type_support, "pico/capteur2");
    if (rc != RCL_RET_OK) return false;
    rc = rclc_publisher_init_default(&calib_publisher3, &node, type_support, "pico/capteur3");
    if (rc != RCL_RET_OK) return false;
#else
    rc = rclc_publisher_init_default(&odom_simple_publisher, &node, type_support, "pico/odom_simple");
    if (rc != RCL_RET_OK)
        return false;
#endif
    auto empty_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    rc = rclc_subscription_init_default(&reset_odom_subscriber, &node, empty_type_support, "pico/reset_odom");
    if (rc != RCL_RET_OK)
        return false;

    executor = rclc_executor_get_zero_initialized_executor();
    // L'exécuteur doit maintenant surveiller 2 abonnements (cmd_simple et reset_odom)
    rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
    if (rc != RCL_RET_OK)
        return false;

    rc = rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
        return false;

    rc = rclc_executor_add_subscription(&executor, &reset_odom_subscriber, &reset_odom_msg, &reset_odom_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
        return false;

    return true;
}

/**
 * DÉBRANCHER LE TÉLÉPHONE : NETTOYAGE MICRO-ROS
 * Utilisé si on perd la connexion avec la Pi 5, pour libérer la mémoire proprement.
 */
void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

#if CALIBRATION_MODE > 0
    rcl_publisher_fini(&calib_publisher1, &node);
    rcl_publisher_fini(&calib_publisher2, &node);
    rcl_publisher_fini(&calib_publisher3, &node);
#else
    rcl_publisher_fini(&odom_simple_publisher, &node);
#endif
    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_subscription_fini(&reset_odom_subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

/**
 * ============================================================================
 * LE 2ÈME CERVEAU (COEUR 1) : LE CARTOGRAPHE
 * ============================================================================
 * Cette fonction tourne dans le processeur n°2 du Pico.
 * Sa seule mission dans la vie : regarder le sol, calculer le déplacement (Odométrie),
 * et s'assurer que les mathématiques tournent à l'heure pile (toutes les 20 ms),
 * sans jamais être dérangé par les moteurs ou la communication.
 */
void core1_entry()
{
    uint32_t last_hw_time = to_ms_since_boot(get_absolute_time());

    // Boucle infinie du Coeur 1
    while (true)
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // C'est l'heure ! (20 ms se sont écoulées)
        if (current_time - last_hw_time >= 20)
        {
            last_hw_time = current_time;

            // 1. On prépare les variables (0 = aucun mouvement)
            int16_t dx1 = 0, dy1 = 0;
            int16_t dx2 = 0, dy2 = 0;
            int16_t dx3 = 0, dy3 = 0;

            // 2. On lit les capteurs optiques.
            // (Timeout de 2ms = Si un capteur est cassé, on abandonne très vite pour ne pas planter le robot)
            capteur1.get_motion(dx1, dy1, 2);
            capteur2.get_motion(dx2, dy2, 2);
            capteur3.get_motion(dx3, dy3, 2);

#if CALIBRATION_MODE == 1
            // MODE CALIBRATION : On additionne simplement les ticks bruts de chaque capteur
            mutex_enter_blocking(&odom_mutex);
            calib_X1 += (float)(invert_dx1 ? -dx1 : dx1);
            calib_Y1 += (float)(invert_dy1 ? -dy1 : dy1);
            calib_X2 += (float)(invert_dx2 ? -dx2 : dx2);
            calib_Y2 += (float)(invert_dy2 ? -dy2 : dy2);
            calib_X3 += (float)(invert_dx3 ? -dx3 : dx3);
            calib_Y3 += (float)(invert_dy3 ? -dy3 : dy3);
            mutex_exit(&odom_mutex);
#elif CALIBRATION_MODE == 2
            // MODE VÉRIFICATION : On convertit les ticks en mètres pour chaque capteur
            float mx1 = C1 * (float)(invert_dx1 ? -dx1 : dx1);
            float my1 = C1 * (float)(invert_dy1 ? -dy1 : dy1);
            float mx2 = C2 * (float)(invert_dx2 ? -dx2 : dx2);
            float my2 = C2 * (float)(invert_dy2 ? -dy2 : dy2);
            float mx3 = C3 * (float)(invert_dx3 ? -dx3 : dx3);
            float my3 = C3 * (float)(invert_dy3 ? -dy3 : dy3);

            mutex_enter_blocking(&odom_mutex);
            calib_mX1 += mx1;
            calib_mY1 += my1;
            calib_mX2 += mx2;
            calib_mY2 += my2;
            calib_mX3 += mx3;
            calib_mY3 += my3;
            mutex_exit(&odom_mutex);
#else
            // MODE NORMAL : On calcule la position globale (Odométrie)
            mutex_enter_blocking(&odom_mutex);
            updateOdometry(dx1, dy1, dx2, dy2, dx3, dy3);
            mutex_exit(&odom_mutex); // Libère l'odom
#endif
        }
        sleep_ms(1); // Petite sieste de 1ms pour ne pas surchauffer le processeur
    }
}

/**
 * ============================================================================
 * LE CERVEAU PRINCIPAL (COEUR 0) : LE CHEF D'ORCHESTRE
 * ============================================================================
 * C'est ici que le robot s'allume (la fonction main).
 * Il lance les moteurs, réveille le 2ème coeur, et gère la boucle principale
 * de sécurité et de discussion avec l'ordinateur.
 */
int main()
{
    // Initialisation de la carte Pico et des périphériques
    stdio_init_all();
    sleep_ms(2000);

#if DEBUG_UART == 1
    printf("--- Test de diagnostic UART du Pico 2 ---\n");
    
    // Configuration de l'UART0 sur les broches GP0 et GP1 à 115200 bauds
    // (Note : Remplacer 0 et 1 par 12 et 13 si tu veux tester la liaison avec la Pi 5)
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    while (true) {
        uart_puts(uart0, "TEST_OK\n");
        sleep_ms(100);
        
        if (uart_is_readable(uart0)) {
            printf("Donnee recue sur RX: ");
            while (uart_is_readable(uart0)) {
                putchar(uart_getc(uart0)); // Affiche chaque caractère reçu
            }
            printf("\n");
        } else {
            printf("ERREUR : Rien ne revient sur RX. TX est peut-etre mort ou RX est bloque.\n");
        }
        sleep_ms(1000);
    }
#endif

    // Préparation des "bâtons de parole"
    mutex_init(&mks_mutex);
    mutex_init(&odom_mutex);

    // Préparation de la petite LED RGB
    gpio_init(LED_R);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_init(LED_B);
    gpio_set_dir(LED_B, GPIO_OUT);
    set_rgb(1, 0, 0); // Rouge au démarrage

    // Initialisation des Moteurs (RS485)
    mks_init();
    sleep_ms(2000);
    uint8_t t[5];
    for (int i = 1; i <= 3; i++)
    {
        // Au démarrage, on s'assure que tous les moteurs sont éteints (en roue libre)
        mks_set_enable(t, i, get_mks_enable_state(i, 0));
        t[4] = mks_get_checksum(t, 4);
        mks_send(t, 5);
        sleep_ms(20);
    }
    set_rgb(1, 0, 1);

    // Indique à micro-ROS qu'on va utiliser l'UART0 (le port série vers la Pi 5)
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    // Initialisation des 3 capteurs optiques
#if CALIBRATION_MODE > 0
    // En mode calibration, on initialise tous les capteurs pour publier leurs données
    capteur1.init();
    capteur2.init();
    capteur3.init();
#else // CALIBRATION_MODE == 0
    capteur1.init();
    capteur1.set_rotation(PAA5100::DEGREES_0);
    capteur2.init();
    capteur2.set_rotation(PAA5100::DEGREES_0);
    capteur3.init();
    capteur3.set_rotation(PAA5100::DEGREES_0);
#endif
    // Lancement officiel du 2ème processeur (Le Cartographe)
    multicore_launch_core1(core1_entry);

    // Préparation des boîtes vides pour les messages
    geometry_msgs__msg__Point32__init(&cmd_vel_msg);
#if CALIBRATION_MODE > 0
    geometry_msgs__msg__Point32__init(&calib_msg1);
    geometry_msgs__msg__Point32__init(&calib_msg2);
    geometry_msgs__msg__Point32__init(&calib_msg3);
#else
    geometry_msgs__msg__Point32__init(&odom_simple_msg);
#endif
    std_msgs__msg__Empty__init(&reset_odom_msg);

    set_rgb(1, 1, 0); // Jaune : En attente de l'Agent Micro-ROS

    uint32_t last_pub_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_ping_time = last_pub_time;

    // LA GRANDE BOUCLE INFINIE DU COEUR 0
    while (true)
    {
        // ====================================================================
        // TÂCHE 1 : PARTAGER L'ODOMÉTRIE (Toutes les 20 ms)
        // ====================================================================
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Si on est bien connecté à la Pi 5...
        if (current_time - last_pub_time >= 20)
        {
            last_pub_time = current_time;

            if (micro_ros_init_successful)
            {
                mutex_enter_blocking(&odom_mutex); // On demande le bâton à notre Coeur 1
#if CALIBRATION_MODE == 1
                calib_msg1.x = calib_X1;
                calib_msg1.y = calib_Y1;
                calib_msg1.z = 0.0f;

                calib_msg2.x = calib_X2;
                calib_msg2.y = calib_Y2;
                calib_msg2.z = 0.0f;

                calib_msg3.x = calib_X3;
                calib_msg3.y = calib_Y3;
                calib_msg3.z = 0.0f;
#elif CALIBRATION_MODE == 2
                calib_msg1.x = calib_mX1;
                calib_msg1.y = calib_mY1;
                calib_msg1.z = 0.0f;

                calib_msg2.x = calib_mX2;
                calib_msg2.y = calib_mY2;
                calib_msg2.z = 0.0f;

                calib_msg3.x = calib_mX3;
                calib_msg3.y = calib_mY3;
                calib_msg3.z = 0.0f;
#else
                odom_simple_msg.x = global_X;
                odom_simple_msg.y = global_Y;
                odom_simple_msg.z = global_Theta;
#endif
                mutex_exit(&odom_mutex);
                // On envoie le message sur le réseau ROS !
#if CALIBRATION_MODE > 0
                rcl_publish(&calib_publisher1, &calib_msg1, NULL);
                rcl_publish(&calib_publisher2, &calib_msg2, NULL);
                rcl_publish(&calib_publisher3, &calib_msg3, NULL);
#else
                rcl_publish(&odom_simple_publisher, &odom_simple_msg, NULL);
#endif
            }
        }

        // ====================================================================
        // TÂCHE 2 : LE CHIEN DE GARDE DES MOTEURS (SÉCURITÉ)
        // ====================================================================
        uint32_t time_since_last_cmd = current_time - last_vel_time;
        if (!DISABLE_SAFETY_TIMEOUTS)
        {
            // CAS 1 : Ça fait 10 secondes qu'on n'a pas reçu d'ordre
            if (motors_enabled && time_since_last_cmd > DISABLE_TIMEOUT_MS)
            {
                mutex_enter_blocking(&mks_mutex);
                if (motors_enabled && (to_ms_since_boot(get_absolute_time()) - last_vel_time > DISABLE_TIMEOUT_MS))
                {
                    uint8_t t[5];
                    for (int i = 1; i <= 3; i++)
                    {
                        // On désactive l'énergie des moteurs pour économiser la batterie
                        mks_set_enable(t, i, get_mks_enable_state(i, 0));
                        t[4] = mks_get_checksum(t, 4);
                        mks_send(t, 5);
                        sleep_ms(20);
                    }
                    motors_enabled = false;
                    motors_moving = false;
                }
                mutex_exit(&mks_mutex);
            }
            // CAS 2 : Ça fait 0.5 seconde qu'on n'a pas reçu d'ordre
            else if (motors_moving && time_since_last_cmd > STOP_TIMEOUT_MS)
            {
                mutex_enter_blocking(&mks_mutex);
                if (motors_moving && (to_ms_since_boot(get_absolute_time()) - last_vel_time > STOP_TIMEOUT_MS))
                {
                    uint8_t trame[52];
                    uint8_t c_data[3][7];
                    mks_init_long_packet(trame);
                    uint8_t ids[3] = {ID_MOT_AVANT, ID_MOT_GAUCHE, ID_MOT_DROIT};
                    bool invs[3] = {inv_m1, inv_m2, inv_m3};

                    for (int i = 0; i < 3; i++)
                    {
                        // On freine immédiatement (vitesse 0) mais on garde le couple (bobines sous tension)

                        uint8_t dir = (1 ^ invs[i]) ? 1 : 0;
                        mks_set_speed(c_data[i], ids[i], dir, 0, vit_accel);
                        mks_add_to_long_packet(trame, i, c_data[i], 6);
                    }
                    trame[51] = mks_get_checksum(trame, 51);
                    mks_send(trame, 52);

                    motors_moving = false;
                }
                mutex_exit(&mks_mutex);
            }
        }

        // ====================================================================
        // TÂCHE 3 : GESTION DE LA CONNEXION ROS
        // ====================================================================
        if (micro_ros_init_successful)
        {
            // On lit le câble série pour voir si de nouveaux messages sont arrivés
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)); // Non bloquant

            // Toutes les 500 ms, on fait un petit "Ping" pour voir si la Pi 5 est toujours vivante
            if (current_time - last_ping_time >= 500)
            {
                // Aïe ! Plus personne au bout du fil. On raccroche.
                last_ping_time = current_time;
                if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK)
                {
                    set_rgb(1, 1, 0); // Jaune : Perte de connexion
                    destroy_entities();
                    micro_ros_init_successful = false;
                }
            }
        }
        else
        {
            // Le robot n'est pas connecté. Il réessaie de "sonner" la Pi 5 toutes les 500 ms.
            if (current_time - last_ping_time >= 500)
            {
                last_ping_time = current_time;
                if (rmw_uros_ping_agent(10, 1) == RMW_RET_OK)
                {
                    // La Pi 5 a répondu ! On branche notre téléphone.
                    micro_ros_init_successful = create_entities();
                    if (micro_ros_init_successful)
                    {
                        set_rgb(0, 1, 0); // Vert : Connecté
                    }
                }
            }
        }
        sleep_ms(1); // Petite sieste pour le Coeur 0
    }
}
