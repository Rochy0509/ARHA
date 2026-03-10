/*
 * gripper_control.c — ARHA Gripper Control
 *
 * Implements the STS_UART HAL callbacks for USART2 (half-duplex)
 * and wraps the STS3215 driver for use by the TCP command handler.
 *
 * Author: Kenneth Martinez
 */

#include "motor/gripper_control.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

/* ─── Hardware handles (declared in main.c) ─── */

extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg1;

/* ─── Red LED (LED_RED defined in main.h) ─── */

static void led_red_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_RED_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_RED_Port, &gpio);
    HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

static void led_red_blink(uint8_t count, uint32_t ms)
{
    for (uint8_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_SET);
        osDelay(ms);
        HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_RESET);
        osDelay(ms);
    }
}

/* ─── STS3215 UART Callbacks ─── */

void STS_UART_Transmit(const uint8_t *buf, uint8_t len)
{
    /* Clear flags and flush RX before transmitting a new command */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | 
                                    UART_CLEAR_PEF  | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);

    HAL_HalfDuplex_EnableTransmitter(&huart2);
    if (HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 10) != HAL_OK) {
        HAL_UART_AbortTransmit(&huart2); /* Recover from timeout/error */
    }

    /* Wait for last byte to fully shift out, with a safety timeout */
    uint32_t start_tick = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start_tick) > 10) { /* 10ms timeout */
            HAL_UART_AbortTransmit(&huart2);
            break;
        }
    }

    /* Immediately enable receiver to catch the swift servo ACK */
    HAL_HalfDuplex_EnableReceiver(&huart2);
}

uint8_t STS_UART_Receive(uint8_t *buf, uint8_t len)
{
    /* If there was leftover noise from TX/bus turnaround, clear it */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | 
                                    UART_CLEAR_PEF  | UART_CLEAR_FEF);

    HAL_StatusTypeDef st = HAL_UART_Receive(&huart2, buf, len, 20);
    
    if (st == HAL_OK) return len;
    
    /* If it timed out, HAL leaves RxState as BUSY_RX forever. Abort it. */
    HAL_UART_AbortReceive(&huart2);
    return 0;
}

/* ─── Baud rate re-init helper ─── */

static void uart_set_baud(uint32_t baud)
{
    /* MUST DeInit first — HAL only calls MspInit when gState == RESET.
     * Without this, HAL_HalfDuplex_Init skips the GPIO/clock setup
     * and PD5 never gets configured as USART2_TX.  This was the root
     * cause of all communication failures. */
    HAL_UART_DeInit(&huart2);

    huart2.Init.BaudRate = baud;
    HAL_HalfDuplex_Init(&huart2);

    /* Disable USART2 IRQ — the CubeMX-generated handler steals RX bytes */
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    /* Clear any pending error flags */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                    UART_CLEAR_PEF  | UART_CLEAR_FEF);

    HAL_HalfDuplex_EnableReceiver(&huart2);
}

/* ─── Gripper API ─── */

STS_Status gripper_init(void)
{
    /* Initialise the red LED for status indication */
    led_red_init();

    /* Common STS3215 baud rates to try */
    static const uint32_t bauds[] = {
        1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400
    };
    static const uint8_t n_bauds = sizeof(bauds) / sizeof(bauds[0]);

    STS_Status st = STS_ERR_TIMEOUT;

    /* Try each baud rate */
    for (uint8_t b = 0; b < n_bauds && st != STS_OK; b++) {
        uart_set_baud(bauds[b]);
        osDelay(10);
        HAL_IWDG_Refresh(&hiwdg1);

        /* Try servo IDs 0–10 at this baud rate */
        for (uint8_t id = 0; id <= 10 && st != STS_OK; id++) {
            STS_SendPacket(id, STS_PING, NULL, 0);
            uint8_t dummy = 0;
            st = STS_ReceivePacket(id, NULL, &dummy);
            HAL_IWDG_Refresh(&hiwdg1);
        }
    }

    if (st != STS_OK) {
        /* Motor NOT found at any baud/ID — solid red LED */
        HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_SET);

        /* Restore default baud rate */
        uart_set_baud(1000000);
        return st;
    }

    /* Motor found — blink red LED 3 times, then turn off */
    led_red_blink(3, 150);
    HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_RESET);

    /* Initialise with default torque */
    return GRIPPER_Init(0);
}

STS_Status gripper_close(uint16_t speed)
{
    return GRIPPER_Close(speed);
}

STS_Status gripper_open(uint16_t speed)
{
    return GRIPPER_Open(speed);
}

STS_Status gripper_move_to(uint16_t position, uint16_t speed)
{
    return GRIPPER_MoveTo(position, speed);
}

STS_Status gripper_read_status(GripperStatus *out)
{
    return GRIPPER_ReadStatus(out);
}

STS_Status gripper_read_position(uint16_t *out)
{
    uint8_t raw[2] = {0};
    STS_Status st = STS_ReadData(GRIPPER_SERVO_ID,
                                  STS_ADDR_PRESENT_POSITION, 2, raw);
    if (st != STS_OK) return st;
    *out = (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    return STS_OK;
}

STS_Status gripper_set_torque(uint8_t torque_pct)
{
    return GRIPPER_SetTorque(torque_pct);
}

STS_Status gripper_ping(void)
{
    return GRIPPER_Ping();
}
