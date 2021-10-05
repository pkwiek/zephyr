/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "platform/nrf_802154_platform_sl_lp_timer.h"

#include <assert.h>
#include <kernel.h>
#include <stdbool.h>

#include "drivers/timer/nrf_rtc_timer.h"

#include "platform/nrf_802154_clock.h"
#include "nrf_802154_sl_utils.h"

static volatile bool m_clock_ready;
static bool m_is_running;
static int32_t m_rtc_channel;
static uint32_t m_in_critical_section;
static uint64_t m_last_scheduled_time;

/*! TODO: handle timer firing too early?
 *  @todo handle timer firing too early?
 */
void rtc_irq_handler(int32_t id, uint64_t target_time, void *user_data)
{
+	(void)user_data;
+
/*-------------------------
 * przerwanie na wysokim priorytecie, wpada tutaj i scheduluje nowego heada albo coś
 * zaraz po headzie, jeżeli obsługa była spóźniona; wpisuje m_is_running = true;
 *-------------------------
 */
	assert(id == m_rtc_channel);

	bool was_running;

	was_running = m_is_running;
	m_is_running = false;
	/* m_is_running jest nadpisany, jest false i nie oczekujemy kolejnego
	 * przerwania, ale jesteśmy w kontekście przerwania od RTC, więc nowe
	 * nam nie grozi
	 */

	__DMB();

	if (was_running) {    /* was_running == true */
		nrf_802154_sl_timer_handler(target_time);
		/* handler przetwarza listę i przetwarza
		 * element, który jest "due", a scheduluje nowy element, jeżeli coś jest do
		 * schedulowania, co ustawia ponownie m_is_running = true; a jeżeli nic nie ma to
		 * m_is_running zostaje na false
		 */
	}
}

void nrf_802154_clock_lfclk_ready(void)
{
	m_clock_ready = true;
}

void nrf_802154_platform_sl_lp_timer_init(void)
{
	m_in_critical_section = 0UL;
	m_is_running = false;

	/* Setup low frequency clock. */
	nrf_802154_clock_lfclk_start();

	while (!m_clock_ready) {
		/* Intentionally empty */
	}

	m_rtc_channel = z_nrf_rtc_timer_chan_alloc();
	if (m_rtc_channel < 0) {
		assert(false);
		return;
	}

	(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);
}

void nrf_802154_platform_sl_lp_timer_deinit(void)
{
	(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);

	z_nrf_rtc_timer_chan_free(m_rtc_channel);

	nrf_802154_clock_lfclk_stop();
}

uint64_t nrf_802154_platform_sl_lptimer_current_lpticks_get(void)
{
	return z_nrf_rtc_timer_read();
}

uint64_t nrf_802154_platform_sl_lptimer_us_to_lpticks_convert(uint64_t us, bool round_up)
{
	if (round_up) {
		return NRF_802154_SL_US_TO_RTC_TICKS(us);
	} else {
		return NRF_802154_SL_US_TO_RTC_TICKS(us) - 1ULL;
	}
}

uint64_t nrf_802154_platform_sl_lptimer_lpticks_to_us_convert(uint64_t lpticks)
{
	/* Calculations are performed on uint64_t as it is safe to assume
	 * overflow will not occur in any foreseeable future.
	 */
	return NRF_802154_SL_RTC_TICKS_TO_US(lpticks);
}

/* TODO: Should schedule_at and disable functions be preemptible by each other?
 * This would mean guarding m_is_running flag.
 */
void nrf_802154_platform_sl_lptimer_schedule_at(uint64_t fire_lpticks);
{
	/* This function is not required to be reentrant, hence no critical section. */
	/*! TODO: handle last ticks equal to fire_lpticks
	 *  @todo handle last ticks equal to fire_lpticks
	 */
	m_last_scheduled_time = fire_lpticks;
	m_is_running = true;
	__DMB();

	z_nrf_rtc_timer_set(m_rtc_channel, fire_lpticks, rtc_irq_handler, NULL);
}

void nrf_802154_platform_sl_lptimer_disable(void);
{
	/* This function is not required to be reentrant, hence no critical section. */
	(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);

	m_is_running = false;
	__DMB();
}

void nrf_802154_platform_sl_lptimer_critical_section_enter(void)
{
	nrf_802154_sl_mcu_critical_state_t state;

	nrf_802154_sl_mcu_critical_enter(state);

	m_in_critical_section++;
	__DMB();

	if (m_in_critical_section == 1UL) {
		(void)z_nrf_rtc_timer_compare_int_lock(m_rtc_channel);
		(void)z_nrf_rtc_timer_overflow_int_lock();
	}

	nrf_802154_sl_mcu_critical_exit(state);
}

void nrf_802154_platform_sl_lptimer_critical_section_exit(void)
{
	nrf_802154_sl_mcu_critical_state_t state;

	nrf_802154_sl_mcu_critical_enter(state);

	/*! TODO: Shall this be an assertion?
	 *  @todo Shall this be an assertion?
	 */
	assert(m_in_critical_section > 0UL);

	z_nrf_rtc_timer_compare_int_unlock(m_rtc_channel,
					   m_is_running && (m_in_critical_section == 1UL));
	z_nrf_rtc_timer_overflow_int_unlock(m_in_critical_section == 1UL);

	m_in_critical_section--;
	__DMB();

	nrf_802154_sl_mcu_critical_exit(state);
}
