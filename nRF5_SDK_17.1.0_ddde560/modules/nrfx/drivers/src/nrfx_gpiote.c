/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <nrfx.h>

#if NRFX_CHECK(NRFX_GPIOTE_ENABLED)

#include <nrfx_gpiote.h>
#include "nrf_bitmask.h"
#include <string.h>

#define NRFX_LOG_MODULE GPIOTE
#include <nrfx_log.h>

#if (GPIO_COUNT == 1)
#define MAX_PIN_NUMBER 32
#elif (GPIO_COUNT == 2)
#define MAX_PIN_NUMBER (32 + P1_PIN_NUM)
#else
#error "Not supported."
#endif

#define FORBIDDEN_HANDLER_ADDRESS ((nrfx_gpiote_evt_handler_t)UINT32_MAX)
#define PIN_NOT_USED              (-1)
#define PIN_USED                  (-2)
#define NO_CHANNELS               (-1)
#define POLARITY_FIELD_POS        (6)
#define POLARITY_FIELD_MASK       (0xC0)

/* Check if every pin can be encoded on provided number of bits. */
NRFX_STATIC_ASSERT(MAX_PIN_NUMBER <= (1 << POLARITY_FIELD_POS));

/**
 * @brief Macro for converting task-event index to an address of an event register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_IDX_TO_EVENT_ADDR(idx)    (nrf_gpiote_events_t)((uint32_t)NRF_GPIOTE_EVENTS_IN_0 + \
                                                           (sizeof(uint32_t) * (idx)))

/**
 * @brief Macro for converting task-event index of OUT task to an address of a task register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_OUT_IDX_TO_TASK_ADDR(idx) (nrf_gpiote_tasks_t)((uint32_t)NRF_GPIOTE_TASKS_OUT_0 + \
                                                          (sizeof(uint32_t) * (idx)))

#if defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__NRFX_DOXYGEN__)
/**
 * @brief Macro for converting task-event index of SET task to an address of a task register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_SET_IDX_TO_TASK_ADDR(idx) (nrf_gpiote_tasks_t)((uint32_t)NRF_GPIOTE_TASKS_SET_0 + \
                                                          (sizeof(uint32_t) * (idx)))

#endif // defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__NRFX_DOXYGEN__)

#if defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__NRFX_DOXYGEN__)
/**
 * @brief Macro for converting task-event index of CLR task to an address of a task register.
 *
 * Macro utilizes the fact that registers are grouped together in ascending order.
 */
#define TE_CLR_IDX_TO_TASK_ADDR(idx) (nrf_gpiote_tasks_t)((uint32_t)NRF_GPIOTE_TASKS_CLR_0 + \
                                                          (sizeof(uint32_t) * (idx)))

#endif // defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__NRFX_DOXYGEN__)

/*lint -save -e571*/ /* Suppress "Warning 571: Suspicious cast" */
typedef struct
{
    nrfx_gpiote_evt_handler_t handlers[GPIOTE_CH_NUM + NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS];
    int8_t                    pin_assignments[MAX_PIN_NUMBER];
    int8_t                    port_handlers_pins[NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS];
    uint8_t                   configured_pins[((MAX_PIN_NUMBER)+7) / 8];
    nrfx_drv_state_t          state;
} gpiote_control_block_t;

static gpiote_control_block_t m_cb;

__STATIC_INLINE bool pin_in_use(uint32_t pin)
{
    return (m_cb.pin_assignments[pin] != PIN_NOT_USED);
}


__STATIC_INLINE bool pin_in_use_as_non_task_out(uint32_t pin)
{
    return (m_cb.pin_assignments[pin] == PIN_USED);
}


__STATIC_INLINE bool pin_in_use_by_te(uint32_t pin)
{
    return (m_cb.pin_assignments[pin] >= 0 && m_cb.pin_assignments[pin] < GPIOTE_CH_NUM) ?
            true : false;
}


__STATIC_INLINE bool pin_in_use_by_port(uint32_t pin)
{
    return (m_cb.pin_assignments[pin] >= GPIOTE_CH_NUM);
}


__STATIC_INLINE bool pin_in_use_by_gpiote(uint32_t pin)
{
    return (m_cb.pin_assignments[pin] >= 0);
}


__STATIC_INLINE void pin_in_use_by_te_set(uint32_t                  pin,
                                          uint32_t                  channel_id,
                                          nrfx_gpiote_evt_handler_t handler,
                                          bool                      is_channel)
{
    m_cb.pin_assignments[pin] = channel_id;
    m_cb.handlers[channel_id] = handler;
    if (!is_channel)
    {
        m_cb.port_handlers_pins[channel_id - GPIOTE_CH_NUM] = (int8_t)pin;
    }
}


__STATIC_INLINE void pin_in_use_set(uint32_t pin)
{
    m_cb.pin_assignments[pin] = PIN_USED;
}


__STATIC_INLINE void pin_in_use_clear(uint32_t pin)
{
    m_cb.pin_assignments[pin] = PIN_NOT_USED;
}


__STATIC_INLINE void pin_configured_set(uint32_t pin)
{
    nrf_bitmask_bit_set(pin, m_cb.configured_pins);
}

__STATIC_INLINE void pin_configured_clear(uint32_t pin)
{
    nrf_bitmask_bit_clear(pin, m_cb.configured_pins);
}

__STATIC_INLINE bool pin_configured_check(uint32_t pin)
{
    return 0 != nrf_bitmask_bit_is_set(pin, m_cb.configured_pins);
}

__STATIC_INLINE int8_t channel_port_get(uint32_t pin)
{
    return m_cb.pin_assignments[pin];
}


__STATIC_INLINE nrfx_gpiote_evt_handler_t channel_handler_get(uint32_t channel)
{
    return m_cb.handlers[channel];
}

static nrfx_gpiote_pin_t port_handler_pin_get(uint32_t handler_idx)
{
    uint8_t pin_and_polarity = (uint8_t)m_cb.port_handlers_pins[handler_idx];
    return (nrfx_gpiote_pin_t)(pin_and_polarity & ~POLARITY_FIELD_MASK);
}

static nrf_gpiote_polarity_t port_handler_polarity_get(uint32_t handler_idx)
{
    uint8_t pin_and_polarity = (uint8_t)m_cb.port_handlers_pins[handler_idx];
    return (nrf_gpiote_polarity_t)((pin_and_polarity & POLARITY_FIELD_MASK) >> POLARITY_FIELD_POS);
}

static int8_t channel_port_alloc(uint32_t pin, nrfx_gpiote_evt_handler_t handler, bool channel)
{
    int8_t   channel_id = NO_CHANNELS;
    uint32_t i;

    uint32_t start_idx = channel ? 0 : GPIOTE_CH_NUM;
    uint32_t end_idx   =
        channel ? GPIOTE_CH_NUM : (GPIOTE_CH_NUM + NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS);

    // critical section

    for (i = start_idx; i < end_idx; i++)
    {
        if (m_cb.handlers[i] == FORBIDDEN_HANDLER_ADDRESS)
        {
            pin_in_use_by_te_set(pin, i, handler, channel);
            channel_id = i;
            break;
        }
    }
    // critical section
    return channel_id;
}


static void channel_free(uint8_t channel_id)
{
    m_cb.handlers[channel_id] = FORBIDDEN_HANDLER_ADDRESS;
    if (channel_id >= GPIOTE_CH_NUM)
    {
        m_cb.port_handlers_pins[channel_id - GPIOTE_CH_NUM] = (int8_t)PIN_NOT_USED;
    }
}


nrfx_err_t nrfx_gpiote_init(void)
{
    nrfx_err_t err_code;

    if (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    uint8_t i;

    for (i = 0; i < MAX_PIN_NUMBER; i++)
    {
        if (nrf_gpio_pin_present_check(i))
        {
            pin_in_use_clear(i);
        }
    }

    for (i = 0; i < (GPIOTE_CH_NUM + NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS); i++)
    {
        channel_free(i);
    }

    memset(m_cb.configured_pins, 0, sizeof(m_cb.configured_pins));

    NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(NRF_GPIOTE), NRFX_GPIOTE_CONFIG_IRQ_PRIORITY);
    NRFX_IRQ_ENABLE(nrfx_get_irq_number(NRF_GPIOTE));
    nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
    nrf_gpiote_int_enable(GPIOTE_INTENSET_PORT_Msk);
    m_cb.state = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}


bool nrfx_gpiote_is_init(void)
{
    return (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED) ? true : false;
}


void nrfx_gpiote_uninit(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    uint32_t i;

    for (i = 0; i < MAX_PIN_NUMBER; i++)
    {   
        if (nrf_gpio_pin_present_check(i))
        {
            if (pin_in_use_as_non_task_out(i))
            {
                nrfx_gpiote_out_uninit(i);
            }
            else if (pin_in_use_by_gpiote(i))
            {
                /* Disable gpiote_in is having the same effect on out pin as gpiote_out_uninit on
                 * so it can be called on all pins used by GPIOTE.
                 */
                nrfx_gpiote_in_uninit(i);
            }
        }
    }
    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}


nrfx_err_t nrfx_gpiote_out_init(nrfx_gpiote_pin_t                pin,
                                nrfx_gpiote_out_config_t const * p_config)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_config);

    nrfx_err_t err_code = NRFX_SUCCESS;

    if (pin_in_use(pin))
    {
        err_code = NRFX_ERROR_INVALID_STATE;
    }
    else
    {
        if (p_config->task_pin)
        {
            int8_t channel = channel_port_alloc(pin, NULL, true);

            if (channel != NO_CHANNELS)
            {
                nrf_gpiote_task_configure((uint32_t)channel,
                                          pin,
                                          p_config->action,
                                          p_config->init_state);
            }
            else
            {
                err_code = NRFX_ERROR_NO_MEM;
            }
        }
        else
        {
            pin_in_use_set(pin);
        }

        if (err_code == NRFX_SUCCESS)
        {
            if (p_config->init_state == NRF_GPIOTE_INITIAL_VALUE_HIGH)
            {
                nrf_gpio_pin_set(pin);
            }
            else
            {
                nrf_gpio_pin_clear(pin);
            }

            nrf_gpio_cfg_output(pin);
            pin_configured_set(pin);
        }
    }

    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}


void nrfx_gpiote_out_uninit(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));

    if (pin_in_use_by_te(pin))
    {
        channel_free((uint8_t)channel_port_get(pin));
        nrf_gpiote_te_default((uint32_t)channel_port_get(pin));
    }
    pin_in_use_clear(pin);

    if (pin_configured_check(pin))
    {
        nrf_gpio_cfg_default(pin);
        pin_configured_clear(pin);
    }
}


void nrfx_gpiote_out_set(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(!pin_in_use_by_te(pin));

    nrf_gpio_pin_set(pin);
}


void nrfx_gpiote_out_clear(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(!pin_in_use_by_te(pin));

    nrf_gpio_pin_clear(pin);
}


void nrfx_gpiote_out_toggle(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(!pin_in_use_by_te(pin));

    nrf_gpio_pin_toggle(pin);
}


void nrfx_gpiote_out_task_enable(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_task_enable((uint32_t)m_cb.pin_assignments[pin]);
}


void nrfx_gpiote_out_task_disable(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_task_disable((uint32_t)m_cb.pin_assignments[pin]);
}


nrf_gpiote_tasks_t nrfx_gpiote_out_task_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    return TE_OUT_IDX_TO_TASK_ADDR((uint32_t)channel_port_get(pin));
}


uint32_t nrfx_gpiote_out_task_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_tasks_t task = nrfx_gpiote_out_task_get(pin);
    return nrf_gpiote_task_addr_get(task);
}


#if defined(GPIOTE_FEATURE_SET_PRESENT)
nrf_gpiote_tasks_t nrfx_gpiote_set_task_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    return TE_SET_IDX_TO_TASK_ADDR((uint32_t)channel_port_get(pin));
}


uint32_t nrfx_gpiote_set_task_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_tasks_t task = nrfx_gpiote_set_task_get(pin);
    return nrf_gpiote_task_addr_get(task);
}
#endif // defined(GPIOTE_FEATURE_SET_PRESENT)


#if defined(GPIOTE_FEATURE_CLR_PRESENT)
nrf_gpiote_tasks_t nrfx_gpiote_clr_task_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    return TE_CLR_IDX_TO_TASK_ADDR((uint32_t)channel_port_get(pin));
}


uint32_t nrfx_gpiote_clr_task_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_tasks_t task = nrfx_gpiote_clr_task_get(pin);
    return nrf_gpiote_task_addr_get(task);
}
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)


void nrfx_gpiote_out_task_force(nrfx_gpiote_pin_t pin, uint8_t state)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_outinit_t init_val =
        state ? NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW;
    nrf_gpiote_task_force((uint32_t)m_cb.pin_assignments[pin], init_val);
}


void nrfx_gpiote_out_task_trigger(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_tasks_t task = TE_OUT_IDX_TO_TASK_ADDR((uint32_t)channel_port_get(pin));
    nrf_gpiote_task_set(task);
}


#if defined(GPIOTE_FEATURE_SET_PRESENT)
void nrfx_gpiote_set_task_trigger(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_tasks_t task = TE_SET_IDX_TO_TASK_ADDR((uint32_t)channel_port_get(pin));
    nrf_gpiote_task_set(task);
}


#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if  defined(GPIOTE_FEATURE_CLR_PRESENT)
void nrfx_gpiote_clr_task_trigger(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_tasks_t task = TE_CLR_IDX_TO_TASK_ADDR((uint32_t)channel_port_get(pin));
    nrf_gpiote_task_set(task);
}


#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

nrfx_err_t nrfx_gpiote_in_init(nrfx_gpiote_pin_t               pin,
                               nrfx_gpiote_in_config_t const * p_config,
                               nrfx_gpiote_evt_handler_t       evt_handler)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    nrfx_err_t err_code = NRFX_SUCCESS;

    /* Only one GPIOTE channel can be assigned to one physical pin. */
    if (pin_in_use_by_gpiote(pin))
    {
        err_code = NRFX_ERROR_INVALID_STATE;
    }
    else
    {
        int8_t channel = channel_port_alloc(pin, evt_handler, p_config->hi_accuracy);
        if (channel != NO_CHANNELS)
        {
            if (!p_config->skip_gpio_setup)
            {
                if (p_config->is_watcher)
                {
                    nrf_gpio_cfg_watcher(pin);
                }
                else
                {
                    nrf_gpio_cfg_input(pin, p_config->pull);
                }
                pin_configured_set(pin);
            }

            if (p_config->hi_accuracy)
            {
                nrf_gpiote_event_configure((uint32_t)channel, pin, p_config->sense);
            }
            else
            {
                m_cb.port_handlers_pins[ channel - GPIOTE_CH_NUM] |= (p_config->sense) <<
                                                                    POLARITY_FIELD_POS;
            }
        }
        else
        {
            err_code = NRFX_ERROR_NO_MEM;
        }
    }

    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

void nrfx_gpiote_in_event_enable(nrfx_gpiote_pin_t pin, bool int_enable)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_gpiote(pin));
    if (pin_in_use_by_port(pin))
    {
        nrf_gpiote_polarity_t polarity =
            port_handler_polarity_get(channel_port_get(pin) - GPIOTE_CH_NUM);
        nrf_gpio_pin_sense_t sense;
        if (polarity == NRF_GPIOTE_POLARITY_TOGGLE)
        {
            /* read current pin state and set for next sense to oposit */
            sense = (nrf_gpio_pin_read(pin)) ?
                    NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
        }
        else
        {
            sense = (polarity == NRF_GPIOTE_POLARITY_LOTOHI) ?
                    NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW;
        }
        nrf_gpio_cfg_sense_set(pin, sense);
    }
    else if (pin_in_use_by_te(pin))
    {
        int32_t             channel = (int32_t)channel_port_get(pin);
        nrf_gpiote_events_t event   = TE_IDX_TO_EVENT_ADDR((uint32_t)channel);

        nrf_gpiote_event_enable((uint32_t)channel);

        nrf_gpiote_event_clear(event);
        if (int_enable)
        {
            nrfx_gpiote_evt_handler_t handler = channel_handler_get((uint32_t)channel_port_get(pin));
            // Enable the interrupt only if event handler was provided.
            if (handler)
            {
                nrf_gpiote_int_enable(1 << channel);
            }
        }
    }
}


void nrfx_gpiote_in_event_disable(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_gpiote(pin));
    if (pin_in_use_by_port(pin))
    {
        nrf_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
    }
    else if (pin_in_use_by_te(pin))
    {
        int32_t channel = (int32_t)channel_port_get(pin);
        nrf_gpiote_event_disable((uint32_t)channel);
        nrf_gpiote_int_disable(1 << channel);
    }
}


void nrfx_gpiote_in_uninit(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_gpiote(pin));
    nrfx_gpiote_in_event_disable(pin);
    if (pin_in_use_by_te(pin))
    {
        nrf_gpiote_te_default((uint32_t)channel_port_get(pin));
    }
    if (pin_configured_check(pin))
    {
        nrf_gpio_cfg_default(pin);
        pin_configured_clear(pin);
    }
    if (pin_in_use_by_gpiote(pin))
    {
        channel_free((uint8_t)channel_port_get(pin));
    }
    pin_in_use_clear(pin);
}


bool nrfx_gpiote_in_is_set(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    return nrf_gpio_pin_read(pin) ? true : false;
}


nrf_gpiote_events_t nrfx_gpiote_in_event_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use_by_port(pin) || pin_in_use_by_te(pin));

    if (pin_in_use_by_te(pin))
    {
        return TE_IDX_TO_EVENT_ADDR((uint32_t)channel_port_get(pin));
    }

    return NRF_GPIOTE_EVENTS_PORT;
}


uint32_t nrfx_gpiote_in_event_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_events_t event = nrfx_gpiote_in_event_get(pin);
    return nrf_gpiote_event_addr_get(event);
}

#if defined(NRF_GPIO_LATCH_PRESENT)
static bool latch_pending_read_and_check(uint32_t * latch)
{
    nrf_gpio_latches_read_and_clear(0, GPIO_COUNT, latch);

    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        if (latch[port_idx])
        {
            /* If any of the latch bits is still set, it means another edge has been captured
             * before or during the interrupt processing. Therefore event-processing loop
             * should be executed again. */
            return true;
        }
    }
    return false;
}

static void port_event_handle(uint32_t * latch)
{
    do {
        for (uint32_t i = 0; i < NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS; i++)
        {
            if (m_cb.port_handlers_pins[i] == PIN_NOT_USED)
            {
                continue;
            }

            /* Process pin further only if LATCH bit associated with this pin was set. */
            nrfx_gpiote_pin_t pin = port_handler_pin_get(i);
            if (nrf_bitmask_bit_is_set(pin, latch))
            {
                nrf_gpiote_polarity_t polarity = port_handler_polarity_get(i);
                nrf_gpio_pin_sense_t sense     = nrf_gpio_pin_sense_get(pin);

                NRFX_LOG_DEBUG("PORT event for pin: %d, polarity: %d.", pin, polarity);

                /* Reconfigure sense to the opposite level, so the internal PINx.DETECT signal
                 * can be deasserted. Therefore PORT event generated again,
                 * unless some other PINx.DETECT signal is still active. */
                nrf_gpio_pin_sense_t next_sense =
                    (sense == NRF_GPIO_PIN_SENSE_HIGH) ? NRF_GPIO_PIN_SENSE_LOW :
                                                         NRF_GPIO_PIN_SENSE_HIGH;
                nrf_gpio_cfg_sense_set(pin, next_sense);

                /* Try to clear LATCH bit corresponding to currently processed pin.
                 * This may not succeed if the pin's state changed during the interrupt processing
                 * and now it matches the new sense configuration. In such case,
                 * the pin will be processed again in another iteration of the outer loop. */
                nrf_gpio_pin_latch_clear(pin);

                /* Invoke user handler only if the sensed pin level
                 * matches its polarity configuration. */
                nrfx_gpiote_evt_handler_t handler =
                    channel_handler_get((uint32_t)channel_port_get(pin));
                if (handler &&
                    ((polarity == NRF_GPIOTE_POLARITY_TOGGLE) ||
                     (sense == NRF_GPIO_PIN_SENSE_HIGH && polarity == NRF_GPIOTE_POLARITY_LOTOHI) ||
                     (sense == NRF_GPIO_PIN_SENSE_LOW && polarity == NRF_GPIOTE_POLARITY_HITOLO)))
                {
                    handler(pin, polarity);
                }
            }
        }
    } while (latch_pending_read_and_check(latch));
}

#else

static bool input_read_and_check(uint32_t * input, uint32_t * pins_to_check)
{
    bool process_inputs_again;
    uint32_t new_input[GPIO_COUNT];

    nrf_gpio_ports_read(0, GPIO_COUNT, new_input);

    process_inputs_again = false;
    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        /* Execute XOR to find out which inputs have changed. */
        uint32_t input_diff = input[port_idx] ^ new_input[port_idx];
        input[port_idx] = new_input[port_idx];
        if (input_diff)
        {
            /* If any differences among inputs were found, mark those pins
             * to be processed again. */
            pins_to_check[port_idx] = input_diff;
            process_inputs_again = true;
        }
        else
        {
            pins_to_check[port_idx] = 0;
        }
    }
    return process_inputs_again;
}

static void port_event_handle(uint32_t * input)
{
    uint32_t pins_to_check[GPIO_COUNT];

    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        pins_to_check[port_idx] = 0xFFFFFFFF;
    }

    do {
        for (uint32_t i = 0; i < NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS; i++)
        {
            if (m_cb.port_handlers_pins[i] == PIN_NOT_USED)
            {
                continue;
            }

            nrfx_gpiote_pin_t pin = port_handler_pin_get(i);
            if (nrf_bitmask_bit_is_set(pin, pins_to_check))
            {
                nrf_gpiote_polarity_t polarity = port_handler_polarity_get(i);
                nrf_gpio_pin_sense_t sense     = nrf_gpio_pin_sense_get(pin);
                bool pin_state                 = nrf_bitmask_bit_is_set(pin, input);

                /* Process pin further only if its state matches its sense level. */
                if ((pin_state && (sense == NRF_GPIO_PIN_SENSE_HIGH)) ||
                    (!pin_state && (sense == NRF_GPIO_PIN_SENSE_LOW)) )
                {
                    /* Reconfigure sense to the opposite level, so the internal PINx.DETECT signal
                     * can be deasserted. Therefore PORT event can be generated again,
                     * unless some other PINx.DETECT signal is still active. */
                    NRFX_LOG_DEBUG("PORT event for pin: %d, polarity: %d.", pin, polarity);
                    nrf_gpio_pin_sense_t next_sense =
                        (sense == NRF_GPIO_PIN_SENSE_HIGH) ? NRF_GPIO_PIN_SENSE_LOW :
                                                             NRF_GPIO_PIN_SENSE_HIGH;
                    nrf_gpio_cfg_sense_set(pin, next_sense);

                    /* Invoke user handler only if the sensed pin level
                     * matches its polarity configuration. */
                    nrfx_gpiote_evt_handler_t handler =
                        channel_handler_get((uint32_t)channel_port_get(pin));
                    if (handler &&
                        ((polarity == NRF_GPIOTE_POLARITY_TOGGLE) ||
                         (sense == NRF_GPIO_PIN_SENSE_HIGH &&
                          polarity == NRF_GPIOTE_POLARITY_LOTOHI) ||
                         (sense == NRF_GPIO_PIN_SENSE_LOW &&
                          polarity == NRF_GPIOTE_POLARITY_HITOLO)))
                    {
                        handler(pin, polarity);
                    }
                }
            }
        }
    } while (input_read_and_check(input, pins_to_check));
}
#endif // defined(NRF_GPIO_LATCH_PRESENT)

void nrfx_gpiote_irq_handler(void)
{
    uint32_t status            = 0;
    uint32_t input[GPIO_COUNT] = {0};

    /* collect status of all GPIOTE pin events. Processing is done once all are collected and cleared.*/
    uint32_t            i;
    nrf_gpiote_events_t event = NRF_GPIOTE_EVENTS_IN_0;
    uint32_t            mask  = (uint32_t)NRF_GPIOTE_INT_IN0_MASK;

    for (i = 0; i < GPIOTE_CH_NUM; i++)
    {
        if (nrf_gpiote_event_is_set(event) && nrf_gpiote_int_is_enabled(mask))
        {
            nrf_gpiote_event_clear(event);
            status |= mask;
        }
        mask <<= 1;
        /* Incrementing to next event, utilizing the fact that events are grouped together
         * in ascending order. */
        event = (nrf_gpiote_events_t)((uint32_t)event + sizeof(uint32_t));
    }

    /* collect PORT status event, if event is set read pins state. Processing is postponed to the
     * end of interrupt. */
    if (nrf_gpiote_event_is_set(NRF_GPIOTE_EVENTS_PORT))
    {
        nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
        status |= (uint32_t)NRF_GPIOTE_INT_PORT_MASK;
#if defined(NRF_GPIO_LATCH_PRESENT)
        nrf_gpio_latches_read_and_clear(0, GPIO_COUNT, input);
#else
        nrf_gpio_ports_read(0, GPIO_COUNT, input);
#endif
    }

    /* Process pin events. */
    if (status & NRF_GPIOTE_INT_IN_MASK)
    {
        mask = (uint32_t)NRF_GPIOTE_INT_IN0_MASK;

        for (i = 0; i < GPIOTE_CH_NUM; i++)
        {
            if (mask & status)
            {
                nrfx_gpiote_pin_t pin = nrf_gpiote_event_pin_get(i);
                NRFX_LOG_DEBUG("Event in number: %d.", i);
                nrf_gpiote_polarity_t        polarity = nrf_gpiote_event_polarity_get(i);
                nrfx_gpiote_evt_handler_t handler  = channel_handler_get(i);
                NRFX_LOG_DEBUG("Pin: %d, polarity: %d.", pin, polarity);
                if (handler)
                {
                    handler(pin, polarity);
                }
            }
            mask <<= 1;
        }
    }

    /* Process PORT event. */
    if (status & (uint32_t)NRF_GPIOTE_INT_PORT_MASK)
    {
        port_event_handle(input);
    }
}


/*lint -restore*/
#endif // NRFX_CHECK(NRFX_GPIOTE_ENABLED)
