#include "Arduino.h"

#include "MotorCtrl.hpp"
#include <DebugDefs.hpp>
#include <FastIO.hpp>

#include <driver/rmt.h>
#include <hal/rmt_ll.h>

#include <driver/timer.h>

#define DSHOT_OUT_PIN 34
#define DSHOT_IN_PIN 33

#define DSHOT_RMT_TX_CHANNEL 0
#define DSHOT_RMT_RX_CHANNEL 4

#define KEEP_ALIVE_TASK_PRIORITY 20

#define LED_R 48
#define LED_B 47

namespace Motor
{
	namespace
	{
		// a buffer that stores the feedback of motor eRPM
		Circbuffer<Motor_FB_t, Motor_FB_buffer_size> Motor_FB_buffer;

		// this works better with Bluejay ESC
		constexpr uint32_t RMT_clock_div = 7;
		// 0 and 1 defs for Dshot 300
		constexpr rmt_item32_t TX_LUT[2] = {{{{14, 0, 24, 1}}}, {{{28, 0, 10, 1}}}};

		// // this works better with BLHeli_S ESC
		// constexpr uint32_t RMT_clock_div = 8;
		// // 0 and 1 defs for Dshot 300
		// constexpr rmt_item32_t TX_LUT[2] = {{{{12, 0, 20, 1}}}, {{{24, 0, 8, 1}}}};

		rmt_config_t TX_channel_config = {
			.rmt_mode = RMT_MODE_TX,
			.channel = static_cast<rmt_channel_t>(DSHOT_RMT_TX_CHANNEL),
			.gpio_num = static_cast<gpio_num_t>(DSHOT_OUT_PIN),
			.clk_div = RMT_clock_div,
			.mem_block_num = 1,
			.flags = 0,
			.tx_config = {
				.carrier_freq_hz = 0,
				.carrier_level = RMT_CARRIER_LEVEL_LOW,
				.idle_level = RMT_IDLE_LEVEL_HIGH,
				.carrier_duty_percent = 0,
				.loop_count = 0,
				.carrier_en = 0,
				.loop_en = 0,
				.idle_output_en = 1}};

		rmt_config_t RX_channel_config = {
			.rmt_mode = RMT_MODE_RX,
			.channel = static_cast<rmt_channel_t>(DSHOT_RMT_RX_CHANNEL),
			.gpio_num = static_cast<gpio_num_t>(DSHOT_IN_PIN),
			.clk_div = RMT_clock_div,
			.mem_block_num = 1,
			.flags = 0,
			.rx_config = {
				.idle_threshold = 3 * (TX_LUT[0].duration0 + TX_LUT[0].duration1),
				.filter_ticks_thresh = 2,
				.filter_en = 1,
				.rm_carrier = 0}};

		/**
		 * @brief Generate rmt_item32_t array for TX
		 *
		 * @param pointer place to store the array
		 * @param throttle throttle value, 0 is lock, 1-47 is special commands, 48-2047 -> 0-2000 throttle
		 *
		 * @note telemetry is obsolete, so here it is always off.
		 *
		 * @warning it is your duty to make sure that throttle is reasonable!
		 */
		void Generate_DShot_item(rmt_item32_t *const pointer, const uint32_t throttle) noexcept
		{
			// // this is for single direction Dshot
			// // please also change the polarity in TX_channel_config
			// static const rmt_item32_t TX_LUT[2] = {{{{6, 1, 10, 0}}}, {{{12, 1, 4, 0}}}};

			uint32_t temp = (throttle << 1);
			temp = (temp << 4) + ((~(temp ^ (temp >> 4) ^ (temp >> 8))) & 0x0F);

			// // this is for single direction Dshot
			// temp = (temp << 4) + ((temp ^ (temp >> 4) ^ (temp >> 8)) & 0x0F);

			for (int i = 0; i < 16; i++)
			{
				pointer[i] = TX_LUT[(temp >> (15 - i)) & 1U];
			}

			pointer[16] = {0};
		}

		// how frequently should we call Keep_alive_task in ms
		constexpr int64_t keep_alive_interval = 50;

		// if we are inside a set throttle cycle
		int32_t in_set_throttle = 0;
		// last time a Dshot command is sent
		int64_t last_Dshot_time = 0;

		TaskHandle_t Keep_alive_task_handle = nullptr;

		int64_t st_rec_time = 0, tx_rec_time = 0, rx_rec_time = 0;
		uint32_t eRPM = 0;

		// a task to keep Dshot alive
		// more convenient than using timer as it is so infrequently called.
		void Keep_alive_task(void *pvParameters)
		{
			while (1)
			{
				vTaskDelay(pdMS_TO_TICKS(keep_alive_interval));

				int64_t st1 = st_rec_time, tx1 = tx_rec_time, rx1 = rx_rec_time;

				if (esp_timer_get_time() - last_Dshot_time >= keep_alive_interval * 1000 && !in_set_throttle)
				{
					Set_throttle(0);
				}
			}
		}

		/**
		 * @brief convert RMT tick length to num of signal
		 *
		 * @param ticks RMT tick length
		 * @return uint32_t -1 for error, 1,2,3 for num, 0 for end
		 */
		int32_t Ticks_to_val(uint32_t ticks)
		{
			// // how many ticks -> 1 num
			// constexpr uint32_t tick_len = 14;
			// // maximum tolerance
			// constexpr uint32_t tick_tol = 4;

			// how many ticks -> 1 num
			constexpr uint32_t tick_len = uint32_t(float(TX_LUT[0].duration0 + TX_LUT[0].duration1) * 0.8F);
			// maximum tolerance
			constexpr uint32_t tick_tol = uint32_t(float(TX_LUT[0].duration0 + TX_LUT[0].duration1) * 0.25F);

			if (tick_len - tick_tol <= ticks && ticks <= tick_len + tick_tol)
			{
				return 1;
			}
			else if (2U * tick_len - tick_tol <= ticks && ticks <= 2U * tick_len + tick_tol)
			{
				return 2;
			}
			else if (3U * tick_len - tick_tol <= ticks && ticks <= 3U * tick_len + tick_tol)
			{
				return 3;
			}
			else if (ticks == 0)
			{
				return 0;
			}

			return -1;
		}

		/**
		 * @brief Parse the DShot feedback into eRPM
		 *
		 * @param RMTptr starting pointer to RMT data
		 * @return int64_t eRPM value in us
		 *
		 * @note if returned value <0, then the read is failed.
		 * 		-1 -> starting voltage error
		 * 		-2 -> tick num error
		 * 		-3 -> data length error
		 * 		-4 -> no GCR code correspondance error
		 * 		-5 -> CRC error
		 */
		int64_t Parse_DShot_Feedback(volatile rmt_item32_t *RMTptr)
		{
			// return if not correct
			if (RMTptr[0].level0 != 0)
			{
				return -1;
			}

			// convert RMT sequence to uint32_t
			uint32_t temp_val = 0, val_len = 0, temp;
			for (int i = 0; i < 20; i++)
			{
				// 0s
				temp = Ticks_to_val(RMTptr[i].duration0);
				if (temp > 0)
				{
					val_len += temp;
					temp_val = temp_val << temp;
				}
				else
				{
					return -2;
				}

				// 1s
				temp = Ticks_to_val(RMTptr[i].duration1);
				if (temp > 0)
				{
					val_len += temp;
					temp_val = ((temp_val + 1) << temp) - 1;
				}
				else if (temp == 0)
				{
					break;
				}
				else
				{
					return -2;
				}
			}

			// because we have 20 switches and there's no symbol that holds
			// twice at the end, we need the total length that start from 0 and
			// end till 0 to be 19,20, or 21.
			if (val_len > 21 || val_len < 19)
			{
				return -3;
			}
			else
			{
				temp_val = ((temp_val + 1) << (21 - val_len)) - 1;
			}

			// decode GCR
			uint32_t gcr = (temp_val ^ (temp_val >> 1));

			// GCR to raw data map
			uint32_t GCRmap[32] = {
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
				0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
				0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF};

			uint32_t origval = 0, crc = 0;

			// crc is the lowest 4 bits
			crc = GCRmap[gcr & 0x1F];
			// if no correspondance exists, return -1
			if (crc > 0xF)
			{
				return -4;
			}
			gcr = (gcr >> 5);

			// real values 12 bit
			for (int i = 0; i < 3; i++)
			{
				uint32_t mpval = GCRmap[gcr & 0x1F];

				// if no correspondance exists, return -1
				if (mpval > 0xF)
				{
					return -4;
				}

				origval = origval + (mpval << (4 * i));
				gcr = (gcr >> 5);
			}

			// check for crc
			if (crc != ((~(origval ^ (origval >> 4) ^ (origval >> 8))) & 0x0F))
			{
				return -5;
			}

			// return value
			return ((origval & 0x1FF) << (origval >> 9));
		}

		rmt_isr_handle_t DShot_ISR_handler = nullptr;
		void IRAM_ATTR DShot_ISR(void *)
		{
			// read RMT interrupt status.
			uint32_t intr_st = RMT.int_st.val;

			// TX end interrupt
			if (intr_st & (1 << DSHOT_RMT_TX_CHANNEL))
			{
				// disable RX first and reset the pointer because there might not be a response last time.
				// and we don't want to trigger just because of the last TX feeding in RX.
				rmt_ll_rx_enable(&RMT, DSHOT_RMT_RX_CHANNEL - 4, false);
				rmt_ll_rx_reset_pointer(&RMT, DSHOT_RMT_RX_CHANNEL - 4);

				// then enable RX
				rmt_ll_rx_enable(&RMT, DSHOT_RMT_RX_CHANNEL - 4, true);

				tx_rec_time = esp_timer_get_time();
			}

			// RX end interrupt
			if (intr_st & (1 << (12 + DSHOT_RMT_RX_CHANNEL)))
			{
				// disable RX, read RX
				rmt_ll_rx_enable(&RMT, DSHOT_RMT_RX_CHANNEL - 4, false);
				rmt_ll_rx_set_mem_owner(&RMT, DSHOT_RMT_RX_CHANNEL - 4, RMT_MEM_OWNER_TX);

				// directly parse Dshot eRPM feedback here
				eRPM = Parse_DShot_Feedback(RMTMEM.chan[DSHOT_RMT_RX_CHANNEL].data32);
				// push to buffer
				Motor_FB_buffer.push(Motor_FB_t{.time = esp_timer_get_time(), .eRPM = eRPM});

				// return the memory ownership to RX
				rmt_ll_rx_set_mem_owner(&RMT, DSHOT_RMT_RX_CHANNEL - 4, RMT_MEM_OWNER_RX);

				rx_rec_time = esp_timer_get_time();
			}

			RMT.int_clr.val = intr_st;
		}

		// transmit a general DShot command
		void DShot_transmit(uint32_t val)
		{
			in_set_throttle = 1;
			last_Dshot_time = esp_timer_get_time();

			static rmt_item32_t RMT_buffer[17];
			Generate_DShot_item(RMT_buffer, val);

			for (size_t i = 0; i < 17U; i++)
			{
				RMTMEM.chan[DSHOT_RMT_TX_CHANNEL].data32[i].val = RMT_buffer[i].val;
			}

			st_rec_time = esp_timer_get_time();

			// disable RX
			rmt_ll_rx_enable(&RMT, DSHOT_RMT_RX_CHANNEL - 4, false);

			// start TX
			rmt_ll_tx_reset_pointer(&RMT, DSHOT_RMT_TX_CHANNEL);
			rmt_ll_tx_start(&RMT, DSHOT_RMT_TX_CHANNEL);

			in_set_throttle = 0;
		}

		int64_t t_startup = 0;
	} // anonymous namespace

	void Init()
	{
		rmt_config(&RX_channel_config);
		rmt_config(&TX_channel_config);

		// disable RX first to prevent overflow in memory
		rmt_ll_rx_enable(&RMT, DSHOT_RMT_RX_CHANNEL - 4, false);

		// startup sequence
		for (int i = 0; i < 1000; i++)
		{
			delayMicroseconds(1000);
			Set_throttle(0);
		}
		for (int i = 0; i < 500; i++)
		{
			delayMicroseconds(1000);
			Set_throttle(200);
		}
		for (int i = 0; i < 500; i++)
		{
			delayMicroseconds(1000);
			Set_throttle(0);
		}

		// wait for last TX to finish
		delayMicroseconds(100);

		// enable TX end and RX end interrupt
		rmt_isr_register(DShot_ISR, NULL, ESP_INTR_FLAG_LEVEL3, &DShot_ISR_handler);
		rmt_ll_enable_tx_end_interrupt(&RMT, DSHOT_RMT_TX_CHANNEL, true);
		rmt_ll_enable_rx_end_interrupt(&RMT, DSHOT_RMT_RX_CHANNEL - 4, true);

		digitalWrite(LED_R, LOW);
		t_startup = esp_timer_get_time();

		// setup the keep alive task
		xTaskCreate(Keep_alive_task,
					"Keep_alive_task",
					5000,
					nullptr,
					KEEP_ALIVE_TASK_PRIORITY,
					&Keep_alive_task_handle);
	}

	void Set_throttle(uint32_t throttle)
	{
		uint32_t throttle_val = ((throttle > DSHOT_THROTTLE_MAX) ? DSHOT_THROTTLE_MAX : throttle) + 48;
		DShot_transmit(throttle_val);
	}

	void Send_command(DShot_Command cmd)
	{
		DShot_transmit(static_cast<uint32_t>(cmd));
	}

	Circbuffer_copycat<Motor_FB_t, Motor_FB_buffer_size> Get_motor_speed_buffer()
	{
		return Circbuffer_copycat<Motor_FB_t, Motor_FB_buffer_size>(&Motor_FB_buffer);
	}
} // namespace Motor