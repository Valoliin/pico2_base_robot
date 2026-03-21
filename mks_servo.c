#include "mks_servo.h"

// --- CONFIGURATION MATÉRIELLE ---
#define UART_ID uart1
#define RS485_DIR_PIN 10
#define MODE_TX 1
#define MODE_RX 0

void mks_init()
{
    // Initialisation UART et GPIO (identique)
    uart_init(UART_ID, 38400);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);
    gpio_init(RS485_DIR_PIN);
    gpio_set_dir(RS485_DIR_PIN, GPIO_OUT);
    gpio_put(RS485_DIR_PIN, MODE_RX);
}

/**
 * FONCTION CALCUL : Remplit les 6 premiers octets de la trame de vitesse
 */
void mks_set_speed(uint8_t *buffer, uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc)
{
    buffer[0] = 0xFA;
    buffer[1] = addr;
    buffer[2] = 0xF6;
    buffer[3] = (dir << 7) | ((speed >> 8) & 0x0F);
    buffer[4] = (uint8_t)(speed & 0xFF);
    buffer[5] = acc;
    // On ne touche pas au CRC ici, l'utilisateur le fera quand il veut
}

/**
 * FONCTION CHECKSUM : Calcule et retourne le CRC d'un buffer
 */
uint8_t mks_get_checksum(uint8_t *buffer, uint8_t size)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < size; i++)
        sum += buffer[i];
    return (uint8_t)(sum & 0xFF);
}

/**
 * FONCTION ENVOI : Gère uniquement le physique (RS485 + UART)
 */
void mks_send(uint8_t *buffer, uint8_t len)
{
    gpio_put(RS485_DIR_PIN, MODE_TX);
    sleep_us(100);
    for (uint8_t i = 0; i < len; i++)
    {
        uart_putc(UART_ID, buffer[i]);
    }
    uart_tx_wait_blocking(UART_ID);
    sleep_ms(1);
    gpio_put(RS485_DIR_PIN, MODE_RX);
}
/**
 * FONCTION RÉCEPTION : Attend le ACK du moteur
 */
bool mks_read_ack()
{
    uint8_t rx_buf[5];
    uint8_t count = 0;
    uint32_t start_time = time_us_32();

    // Timeout de 100ms
    while (time_us_32() - start_time < 100000)
    {
        if (uart_is_readable(UART_ID))
        {
            uint8_t byte = uart_getc(UART_ID);

            if (count == 0 && byte == 0xFB)
            { // Détection Header réponse
                rx_buf[count++] = byte;
            }
            else if (count > 0)
            {
                rx_buf[count++] = byte;
            }

            if (count == 5)
            { // Trame complète reçue
                uint8_t cs = mks_get_checksum(rx_buf, 4);
                if (cs == rx_buf[4])
                {
                    return (rx_buf[3] == 0x01); // Status = 1 signifie succès
                }
                return false;
            }
        }
    }
    return false; // Timeout
}

/**
 * Commande F3 : Enable / Disable
 * Trame totale : 5 octets
 */
void mks_set_enable(uint8_t *buffer, uint8_t addr, uint8_t enable)
{
    buffer[0] = 0xFA;
    buffer[1] = addr;
    buffer[2] = 0xF3;
    buffer[3] = enable & 0x01; // Sécurité : on s'assure que c'est bien 0 ou 1
    // Le CRC ira dans buffer[4]
}

/**
 * Commande F7 : Emergency Stop
 * Trame totale : 4 octets
 */
void mks_set_emergency_stop(uint8_t *buffer, uint8_t addr)
{
    buffer[0] = 0xFA;
    buffer[1] = addr;
    buffer[2] = 0xF7;
    // Le CRC ira dans buffer[3]
}

/**
 * Commande F6 (Vitesse 0) : Arrêt contrôlé
 * Trame totale : 7 octets
 */
void mks_set_stop(uint8_t *buffer, uint8_t addr, uint8_t acc)
{
    buffer[0] = 0xFA;
    buffer[1] = addr;
    buffer[2] = 0xF6;
    buffer[3] = 0x00; // Direction 0 + Vitesse High 0
    buffer[4] = 0x00; // Vitesse Low 0
    buffer[5] = acc;
    // Le CRC ira dans buffer[6]
}