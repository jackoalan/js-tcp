#pragma once

#include "options.h"
#include "pnet_api.h"

class pnet_if;
struct joystick_state;
struct talon_srx;
class can_if;

class pnet_if {
  pnet_t *m_pnet = nullptr;
  struct cfg : pnet_cfg_t {
    explicit cfg(pnet_if *pnet, const char *netif);
  } m_cfg;

  uint32_t m_slot_bitmask = 0;
  uint32_t m_main_arep = UINT32_MAX;
  bool m_error = false;
  bool m_needs_app_ready = false;

  friend struct pnet_callbacks;

  int state(uint32_t arep, pnet_event_values_t state);

  int connect(uint32_t arep, pnet_result_t *p_result);

  int release(uint32_t arep, pnet_result_t *p_result);

  int dcontrol(uint32_t arep, pnet_control_command_t control_command,
               pnet_result_t *p_result);

  int ccontrol(uint32_t arep, pnet_result_t *p_result);

  int read(uint32_t arep, uint32_t api, uint16_t slot, uint16_t subslot,
           uint16_t idx, uint16_t sequence_number,
           uint8_t **pp_read_data,  /**< Out: A pointer to the data */
           uint16_t *p_read_length, /**< Out: Size of data */
           pnet_result_t *p_result);

  int write(uint32_t arep, uint32_t api, uint16_t slot, uint16_t subslot,
            uint16_t idx, uint16_t sequence_number, uint16_t write_length,
            uint8_t *p_write_data, pnet_result_t *p_result);

  int exp_module(uint32_t api, uint16_t slot, uint32_t module_ident);

  int exp_submodule(uint32_t api, uint16_t slot, uint16_t subslot,
                    uint32_t module_ident, uint32_t submodule_ident);

  int new_data_status(
      uint32_t arep, uint32_t crep,
      uint8_t changes, /**< Only modified bits from pnet_data_status_bits_t */
      uint8_t data_status);

  int alarm_ind(uint32_t arep, const pnet_alarm_argument_t *p_alarm_argument,
                uint16_t data_len, uint16_t data_usi, uint8_t *p_data);

  int alarm_cnf(uint32_t arep, pnet_pnio_status_t *p_pnio_status);

  int alarm_ack_cnf(uint32_t arep, int res);

  int reset(bool should_reset_application, uint16_t reset_mode);

  int signal_led(bool led_state);

  int plug_module(uint16_t slot, uint32_t module_ident);

  int plug_submodule(uint16_t slot, uint16_t subslot, uint32_t module_ident,
                     uint32_t submodule_ident, pnet_submodule_dir_t dir,
                     uint16_t length_input, uint16_t length_output);

  void plug_dap();

public:
  explicit pnet_if(const char *netif, uint32_t tick_us);
  bool valid() const { return m_pnet != nullptr; }

  bool periodic(
#ifdef TIBERIUS_CAN
      can_if &can
#endif
      );
#ifdef TIBERIUS_JS
  void send_updates(const joystick_state &state);
#endif
#ifdef TIBERIUS_CAN
  void send_updates(const talon_srx &talon);
#endif
};
