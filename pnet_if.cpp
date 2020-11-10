//
// Created by jacko on 10/21/20.
//

#include "pnet_if.h"
#include "can_if.h"
#include "joystick_state.h"
#include "osal.h"
#include "pnal.h"
#include <cstring>
#include <iostream>
#include <libnet.h>
#include <sys/stat.h>
#include <unistd.h>

constexpr struct station_info {
  const char *device_string = "Tiberius";
#ifdef TIBERIUS_JS
  const char *station_name = "joystick.pilot";
  os_ipaddr_t default_ip = 0x0A0A001E;
  const char *tmp_dir = "/pnet-js";
  const char *serial_number = "00001";
  uint32_t dap_mod_ident = 1;
  uint32_t dev_module_ident = 0x00000030;
  uint32_t dev_submodule_ident = 0x00000001;
  uint16_t dev_max_slot = 1;
  pnet_submodule_dir_t dev_dir = PNET_DIR_INPUT;
  uint16_t dev_length_input = sizeof(joystick_state);
  uint16_t dev_length_output = 0;
#elif defined(TIBERIUS_CAN)
  const char *station_name = "can.robot";
  os_ipaddr_t default_ip = 0x0A14001E;
  const char *tmp_dir = "/pnet-can";
  const char *serial_number = "00002";
  uint32_t dap_mod_ident = 2;
  uint32_t dev_module_ident = 0x00000031;
  uint32_t dev_submodule_ident = 0x00000001;
  uint16_t dev_max_slot = 4;
  pnet_submodule_dir_t dev_dir = PNET_DIR_IO;
  uint16_t dev_length_input = sizeof(can_input_state);
  uint16_t dev_length_output = sizeof(can_output_state);
#endif
} g_station_info;

struct pnet_callbacks {
  static int state(pnet_t *, void *arg, uint32_t arep,
                   pnet_event_values_t state) {
    return reinterpret_cast<pnet_if *>(arg)->state(arep, state);
  }

  static int connect(pnet_t *, void *arg, uint32_t arep,
                     pnet_result_t *p_result) {
    return reinterpret_cast<pnet_if *>(arg)->connect(arep, p_result);
  }

  static int release(pnet_t *, void *arg, uint32_t arep,
                     pnet_result_t *p_result) {
    return reinterpret_cast<pnet_if *>(arg)->release(arep, p_result);
  }

  static int dcontrol(pnet_t *, void *arg, uint32_t arep,
                      pnet_control_command_t control_command,
                      pnet_result_t *p_result) {
    return reinterpret_cast<pnet_if *>(arg)->dcontrol(arep, control_command,
                                                      p_result);
  }

  static int ccontrol(pnet_t *, void *arg, uint32_t arep,
                      pnet_result_t *p_result) {
    return reinterpret_cast<pnet_if *>(arg)->ccontrol(arep, p_result);
  }

  static int read(pnet_t *, void *arg, uint32_t arep, uint32_t api,
                  uint16_t slot, uint16_t subslot, uint16_t idx,
                  uint16_t sequence_number,
                  uint8_t **pp_read_data,  /**< Out: A pointer to the data */
                  uint16_t *p_read_length, /**< Out: Size of data */
                  pnet_result_t *p_result) {
    return reinterpret_cast<pnet_if *>(arg)->read(arep, api, slot, subslot, idx,
                                                  sequence_number, pp_read_data,
                                                  p_read_length, p_result);
  }

  static int write(pnet_t *, void *arg, uint32_t arep, uint32_t api,
                   uint16_t slot, uint16_t subslot, uint16_t idx,
                   uint16_t sequence_number, uint16_t write_length,
                   const uint8_t *p_write_data, pnet_result_t *p_result) {
    return reinterpret_cast<pnet_if *>(arg)->write(
        arep, api, slot, subslot, idx, sequence_number, write_length,
        p_write_data, p_result);
  }

  static int exp_module(pnet_t *, void *arg, uint32_t api, uint16_t slot,
                        uint32_t module_ident) {
    return reinterpret_cast<pnet_if *>(arg)->exp_module(api, slot,
                                                        module_ident);
  }

  static int exp_submodule(pnet_t *, void *arg, uint32_t api, uint16_t slot,
                           uint16_t subslot, uint32_t module_ident,
                           uint32_t submodule_ident, const pnet_data_cfg *cfg) {
    return reinterpret_cast<pnet_if *>(arg)->exp_submodule(
        api, slot, subslot, module_ident, submodule_ident, cfg);
  }

  static int new_data_status(
      pnet_t *, void *arg, uint32_t arep, uint32_t crep,
      uint8_t changes, /**< Only modified bits from pnet_data_status_bits_t */
      uint8_t data_status) {
    return reinterpret_cast<pnet_if *>(arg)->new_data_status(
        arep, crep, changes, data_status);
  }

  static int alarm_ind(pnet_t *, void *arg, uint32_t arep,
                       const pnet_alarm_argument_t *p_alarm_argument,
                       uint16_t data_len, uint16_t data_usi,
                       const uint8_t *p_data) {
    return reinterpret_cast<pnet_if *>(arg)->alarm_ind(
        arep, p_alarm_argument, data_len, data_usi, p_data);
  }

  static int alarm_cnf(pnet_t *, void *arg, uint32_t arep,
                       const pnet_pnio_status_t *p_pnio_status) {
    return reinterpret_cast<pnet_if *>(arg)->alarm_cnf(arep, p_pnio_status);
  }

  static int alarm_ack_cnf(pnet_t *, void *arg, uint32_t arep, int res) {
    return reinterpret_cast<pnet_if *>(arg)->alarm_ack_cnf(arep, res);
  }

  static int reset(pnet_t *, void *arg, bool should_reset_application,
                   uint16_t reset_mode) {
    return reinterpret_cast<pnet_if *>(arg)->reset(should_reset_application,
                                                   reset_mode);
  }

  static int signal_led(pnet_t *, void *arg, bool led_state) {
    return reinterpret_cast<pnet_if *>(arg)->signal_led(led_state);
  }
};

static int get_macaddress(const char *interface_name, os_ethaddr_t *mac_addr) {
  int fd;
  int ret = 0;
  struct ifreq ifr;

  fd = socket(AF_INET, SOCK_DGRAM, 0);

  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);

  ret = ioctl(fd, SIOCGIFHWADDR, &ifr);
  if (ret == 0) {
    memcpy(mac_addr->addr, ifr.ifr_hwaddr.sa_data, 6);
  }
  close(fd);
  return ret;
}

static os_ipaddr_t get_ip_address(const char *interface_name) {
  int fd;
  struct ifreq ifr {};
  os_ipaddr_t ip;

  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
  ioctl(fd, SIOCGIFADDR, &ifr);
  ip = ntohl(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr);
  close(fd);

  return ip;
}

static os_ipaddr_t get_netmask(const char *interface_name) {
  int fd;
  struct ifreq ifr {};
  os_ipaddr_t netmask;

  fd = socket(AF_INET, SOCK_DGRAM, 0);

  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
  ioctl(fd, SIOCGIFNETMASK, &ifr);
  netmask = ntohl(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr);
  close(fd);

  return netmask;
}

static os_ipaddr_t get_gateway(const char *interface_name) {
  /* TODO Read the actual default gateway (somewhat complicated) */

  os_ipaddr_t ip;
  os_ipaddr_t gateway;

  ip = get_ip_address(interface_name);
  if (ip == 0)
    return 0;
  gateway = (ip & 0xFFFFFF00) | 0x00000001;

  return gateway;
}

pnet_if::cfg::cfg(pnet_if *pnet, const char *netif) {
  std::memset(this, 0, sizeof(*this));

  /* Call-backs */
  state_cb = &pnet_callbacks::state;
  connect_cb = &pnet_callbacks::connect;
  release_cb = &pnet_callbacks::release;
  dcontrol_cb = &pnet_callbacks::dcontrol;
  ccontrol_cb = &pnet_callbacks::ccontrol;
  read_cb = &pnet_callbacks::read;
  write_cb = &pnet_callbacks::write;
  exp_module_cb = &pnet_callbacks::exp_module;
  exp_submodule_cb = &pnet_callbacks::exp_submodule;
  new_data_status_cb = &pnet_callbacks::new_data_status;
  alarm_ind_cb = &pnet_callbacks::alarm_ind;
  alarm_cnf_cb = &pnet_callbacks::alarm_cnf;
  alarm_ack_cnf_cb = &pnet_callbacks::alarm_ack_cnf;
  reset_cb = &pnet_callbacks::reset;
  signal_led_cb = &pnet_callbacks::signal_led;
  cb_arg = pnet;

  /* Identification & Maintenance */
  im_0_data.im_vendor_id_hi = 0xde;
  im_0_data.im_vendor_id_lo = 0xad;
  im_0_data.im_hardware_revision = 1;
  im_0_data.im_sw_revision_prefix = 'V'; /* 'V', 'R', 'P', 'U', or 'T' */
  im_0_data.im_sw_revision_functional_enhancement = 0;
  im_0_data.im_sw_revision_bug_fix = 0;
  im_0_data.im_sw_revision_internal_change = 0;
  im_0_data.im_revision_counter = 0; /* Only 0 allowed according to standard */
  im_0_data.im_profile_id = 0x1234;
  im_0_data.im_profile_specific_type = 0x5678;
  im_0_data.im_version_major = 1;
  im_0_data.im_version_minor = 1;
  im_0_data.im_supported = PNET_SUPPORTED_IM1 | PNET_SUPPORTED_IM2 |
                           PNET_SUPPORTED_IM3 | PNET_SUPPORTED_IM4;
  std::strcpy(im_1_data.im_tag_function, "my function");
  std::strcpy(im_1_data.im_tag_location, "my location");
  std::strcpy(im_2_data.im_date, "2020-09-03 13:53");
  std::strcpy(im_3_data.im_descriptor, "my descriptor");
  std::strcpy(im_4_data.im_signature, ""); /* Functional safety */

  /* Device configuration */
  device_id.vendor_id_hi = 0xde;
  device_id.vendor_id_lo = 0xad;
  device_id.device_id_hi = 0xca;
  device_id.device_id_lo = 0x75;
  oem_device_id.vendor_id_hi = 0xc0;
  oem_device_id.vendor_id_lo = 0xff;
  oem_device_id.device_id_hi = 0xee;
  oem_device_id.device_id_lo = 0x01;
  std::strcpy(device_vendor, g_station_info.device_string);
  std::strcpy(manufacturer_specific_string, g_station_info.device_string);
  std::strcpy(station_name, g_station_info.station_name);

  /* Timing */
  min_device_interval = 32; /* Corresponds to 1 ms */

  /* LLDP settings */
  std::strcpy(lldp_cfg.port_id, "port-001");
  lldp_cfg.ttl = PNET_LLDP_TTL;
  lldp_cfg.rtclass_2_status = 0;
  lldp_cfg.rtclass_3_status = 0;
  lldp_cfg.cap_aneg = PNET_LLDP_AUTONEG_SUPPORTED | PNET_LLDP_AUTONEG_ENABLED;
  lldp_cfg.cap_phy = PNET_LLDP_AUTONEG_CAP_100BaseTX_HALF_DUPLEX |
                     PNET_LLDP_AUTONEG_CAP_100BaseTX_FULL_DUPLEX;
  lldp_cfg.mau_type = PNET_MAU_COPPER_100BaseTX_FULL_DUPLEX;

  /* Network configuration */
  send_hello = true;
  dhcp_enable = false;

  os_ethaddr_t macbuffer;
  int ret = get_macaddress(netif, &macbuffer);
  if (ret != 0) {
    printf("Error: The given Ethernet interface does not exist: %s\n", netif);
    exit(EXIT_FAILURE);
  }

  // os_ipaddr_t ip = get_ip_address(netif);
  // os_ipaddr_t netmask = get_netmask(netif);
  // os_ipaddr_t gateway = get_gateway(netif);
  os_ipaddr_t ip = g_station_info.default_ip;
  os_ipaddr_t netmask = 0xFF000000;
  os_ipaddr_t gateway = 0;
  // if (gateway == 0) {
  //  printf("Error: Invalid gateway IP address for Ethernet interface: %s\n",
  //         netif);
  //  exit(EXIT_FAILURE);
  //}

  auto copy_ip_to_struct = [](pnet_cfg_ip_addr_t &destination_struct,
                              os_ipaddr_t ip) {
    destination_struct.a = ((ip >> 24) & 0xFF);
    destination_struct.b = ((ip >> 16) & 0xFF);
    destination_struct.c = ((ip >> 8) & 0xFF);
    destination_struct.d = (ip & 0xFF);
  };

  /* Prepare stack config with IP address, gateway, station name etc */
  strcpy(im_0_data.im_order_id, "12345");
  strcpy(im_0_data.im_serial_number, g_station_info.serial_number);
  copy_ip_to_struct(ip_addr, ip);
  copy_ip_to_struct(ip_gateway, gateway);
  copy_ip_to_struct(ip_mask, netmask);
  const char *tmpdir = getenv("TMPDIR");
  if (!tmpdir)
    tmpdir = "/tmp";
  strncpy(file_directory, tmpdir, 128);
  strncat(file_directory, g_station_info.tmp_dir, 128);
  mkdir(file_directory, 755);
  std::cout << "Storage directory: " << file_directory << std::endl;
  memcpy(eth_addr.addr, macbuffer.addr, sizeof(os_ethaddr_t));
}

int pnet_if::state(uint32_t arep, pnet_event_values_t state) {
  std::cout << "state " << state << std::endl;

  if (state == PNET_EVENT_ABORT) {
    uint16_t err_cls = 0;
    uint16_t err_code = 0;
    if (pnet_get_ar_error_codes(m_pnet, arep, &err_cls, &err_code) == 0) {
      /* A few of the most common error codes */
      switch (err_cls) {
      default:
        std::cerr << "Unknown error class " << err_cls << std::endl;
        break;
      case PNET_ERROR_CODE_1_RTA_ERR_CLS_PROTOCOL:
        switch (err_code) {
        case PNET_ERROR_CODE_2_ABORT_AR_CONSUMER_DHT_EXPIRED:
          std::cerr << "AR_CONSUMER_DHT_EXPIRED" << std::endl;
          break;
        case PNET_ERROR_CODE_2_ABORT_AR_CMI_TIMEOUT:
          std::cerr << "ABORT_AR_CMI_TIMEOUT" << std::endl;
          break;
        case PNET_ERROR_CODE_2_ABORT_AR_RELEASE_IND_RECEIVED:
          std::cerr << "Controller sent release request." << std::endl;
          break;
        default:
          std::cerr << "Unknown error code " << err_code << std::endl;
          break;
        }
        break;
      }
    } else {
      std::cerr << "No error status available" << std::endl;
    }
    /* Only abort AR with correct session key */
    // m_error = true;
    m_main_arep = UINT32_MAX;
  } else if (state == PNET_EVENT_PRMEND) {
    /* Save the arep for later use */
    m_main_arep = arep;
    m_needs_app_ready = true;

    /* Set IOPS for DAP slot (has same numbering as the module identifiers) */
    pnet_input_set_data_and_iops(m_pnet, 0, PNET_SLOT_DAP_IDENT,
                                 PNET_SUBMOD_DAP_IDENT, nullptr, 0,
                                 PNET_IOXS_GOOD);
    pnet_input_set_data_and_iops(m_pnet, 0, PNET_SLOT_DAP_IDENT,
                                 PNET_SUBSLOT_DAP_INTERFACE_1_IDENT, nullptr, 0,
                                 PNET_IOXS_GOOD);
    pnet_input_set_data_and_iops(m_pnet, 0, PNET_SLOT_DAP_IDENT,
                                 PNET_SUBSLOT_DAP_INTERFACE_1_PORT_0_IDENT,
                                 nullptr, 0, PNET_IOXS_GOOD);

    /* Set initial data and IOPS for custom input modules, and IOCS for custom
     * output modules */
#ifdef TIBERIUS_JS
    joystick_state default_state{};
#elif defined(TIBERIUS_CAN)
    can_input_state default_state{};
#endif

    for (uint16_t slot = 1; slot <= g_station_info.dev_max_slot; ++slot) {
      if ((m_slot_bitmask & (1u << slot)) != 0) {
        pnet_input_set_data_and_iops(m_pnet, 0, slot, 1,
                                     (uint8_t *)&default_state,
                                     sizeof(default_state), PNET_IOXS_GOOD);
#ifdef TIBERIUS_CAN
        pnet_output_set_iocs(m_pnet, 0, slot, 1, PNET_IOXS_GOOD);
#endif
      }
    }

    pnet_set_provider_state(m_pnet, true);
  }

  return 0;
}

int pnet_if::connect(uint32_t arep, pnet_result_t *p_result) {
  std::cout << "connect" << std::endl;
  return 0;
}

int pnet_if::release(uint32_t arep, pnet_result_t *p_result) {
  std::cout << "release" << std::endl;
  return 0;
}

int pnet_if::dcontrol(uint32_t arep, pnet_control_command_t control_command,
                      pnet_result_t *p_result) {
  std::cout << "dcontrol" << std::endl;
  return 0;
}

int pnet_if::ccontrol(uint32_t arep, pnet_result_t *p_result) {
  std::cout << "ccontrol" << std::endl;
  return 0;
}

int pnet_if::read(uint32_t arep, uint32_t api, uint16_t slot, uint16_t subslot,
                  uint16_t idx, uint16_t sequence_number,
                  uint8_t **pp_read_data,  /**< Out: A pointer to the data */
                  uint16_t *p_read_length, /**< Out: Size of data */
                  pnet_result_t *p_result) {
  std::cout << "read" << std::endl;
  return 0;
}

int pnet_if::write(uint32_t arep, uint32_t api, uint16_t slot, uint16_t subslot,
                   uint16_t idx, uint16_t sequence_number,
                   uint16_t write_length, const uint8_t *p_write_data,
                   pnet_result_t *p_result) {
  std::cout << "write" << std::endl;
  return 0;
}

int pnet_if::exp_module(uint32_t api, uint16_t slot, uint32_t module_ident) {
  std::cout << "exp_module" << std::endl;

  if (module_ident == g_station_info.dap_mod_ident) {
    return plug_module(slot, g_station_info.dap_mod_ident);
  }

  if (module_ident != g_station_info.dev_module_ident) {
    std::cerr << "unsupported module " << module_ident << std::endl;
    return -1;
  }

  if (slot != 1) {
    std::cerr << "unsupported slot " << slot << std::endl;
    return -1;
  }

  int ret = plug_module(slot, module_ident);
  if (ret == 0) {
    std::cout << "plugged module slot " << slot << std::endl;
  }

  return ret;
}

int pnet_if::exp_submodule(uint32_t api, uint16_t slot, uint16_t subslot,
                           uint32_t module_ident, uint32_t submodule_ident,
                           const pnet_data_cfg *cfg) {
  std::cout << "exp_submodule" << std::endl;

  if (module_ident == g_station_info.dap_mod_ident) {
    switch (submodule_ident) {
    case PNET_SUBMOD_DAP_IDENT:
      return plug_submodule(PNET_SLOT_DAP_IDENT, PNET_SUBMOD_DAP_IDENT,
                            g_station_info.dap_mod_ident, PNET_SUBMOD_DAP_IDENT,
                            PNET_DIR_NO_IO, 0, 0);
    case PNET_SUBMOD_DAP_INTERFACE_1_IDENT:
      return plug_submodule(
          PNET_SLOT_DAP_IDENT, PNET_SUBSLOT_DAP_INTERFACE_1_IDENT,
          g_station_info.dap_mod_ident, PNET_SUBMOD_DAP_INTERFACE_1_IDENT,
          PNET_DIR_NO_IO, 0, 0);
    case PNET_SUBMOD_DAP_INTERFACE_1_PORT_0_IDENT:
      return plug_submodule(
          PNET_SLOT_DAP_IDENT, PNET_SUBSLOT_DAP_INTERFACE_1_PORT_0_IDENT,
          g_station_info.dap_mod_ident,
          PNET_SUBMOD_DAP_INTERFACE_1_PORT_0_IDENT, PNET_DIR_NO_IO, 0, 0);
    default:
      return -1;
    }
  }

  if (module_ident != g_station_info.dev_module_ident) {
    std::cerr << "unsupported module " << module_ident << std::endl;
    return -1;
  }

  if (submodule_ident != g_station_info.dev_submodule_ident) {
    std::cerr << "unsupported submodule " << submodule_ident << std::endl;
    return -1;
  }

  if (slot > g_station_info.dev_max_slot) {
    std::cerr << "unsupported slot " << slot << std::endl;
    return -1;
  }

  if (subslot != 1) {
    std::cerr << "unsupported subslot " << subslot << std::endl;
    return -1;
  }

  int ret = plug_submodule(
      slot, subslot, module_ident, submodule_ident, g_station_info.dev_dir,
      g_station_info.dev_length_input, g_station_info.dev_length_output);
  if (ret == 0) {
    std::cout << "plugged module slot " << slot << std::endl;
    m_slot_bitmask |= 1u << slot;
  }

  return ret;
}

int pnet_if::new_data_status(
    uint32_t arep, uint32_t crep,
    uint8_t changes, /**< Only modified bits from pnet_data_status_bits_t */
    uint8_t data_status) {
  std::cout << "new_data_status" << std::endl;
  return 0;
}

int pnet_if::alarm_ind(uint32_t arep,
                       const pnet_alarm_argument_t *p_alarm_argument,
                       uint16_t data_len, uint16_t data_usi,
                       const uint8_t *p_data) {
  std::cout << "alarm_ind" << std::endl;
  return 0;
}

int pnet_if::alarm_cnf(uint32_t arep, const pnet_pnio_status_t *p_pnio_status) {
  std::cout << "alarm_cnf" << std::endl;
  return 0;
}

int pnet_if::alarm_ack_cnf(uint32_t arep, int res) {
  std::cout << "alarm_ack_cnf" << std::endl;
  return 0;
}

int pnet_if::reset(bool should_reset_application, uint16_t reset_mode) {
  std::cout << "reset" << std::endl;
  return 0;
}

int pnet_if::signal_led(bool led_state) {
  std::cout << "signal_led" << std::endl;
#ifdef TIBERIUS_CAN
  char *outputcommand;
  int textlen = -1;
  int status = -1;

  textlen = asprintf (
     &outputcommand,
     "./set_profinet_leds_linux 0 %u",
     led_state);
  if (textlen < 0) {
    return -1;
  }

  status = system(outputcommand);
  free(outputcommand);
  if (status != 0) {
    std::cout << "failed to set LED state" << std::endl;
    return -1;
  }
#endif
  return 0;
}

int pnet_if::plug_module(uint16_t slot, uint32_t module_ident) {
  pnet_pull_module(m_pnet, 0, slot);
  return pnet_plug_module(m_pnet, 0, slot, module_ident);
}

int pnet_if::plug_submodule(uint16_t slot, uint16_t subslot,
                            uint32_t module_ident, uint32_t submodule_ident,
                            pnet_submodule_dir_t dir, uint16_t length_input,
                            uint16_t length_output) {
  pnet_pull_submodule(m_pnet, 0, slot, subslot);
  return pnet_plug_submodule(m_pnet, 0, slot, subslot, module_ident,
                             submodule_ident, dir, length_input, length_output);
}

void pnet_if::plug_dap() {
  plug_module(PNET_SLOT_DAP_IDENT, g_station_info.dap_mod_ident);
  plug_submodule(PNET_SLOT_DAP_IDENT, PNET_SUBMOD_DAP_IDENT,
                 g_station_info.dap_mod_ident, PNET_SUBMOD_DAP_IDENT,
                 PNET_DIR_NO_IO, 0, 0);
  plug_submodule(PNET_SLOT_DAP_IDENT, PNET_SUBSLOT_DAP_INTERFACE_1_IDENT,
                 g_station_info.dap_mod_ident,
                 PNET_SUBMOD_DAP_INTERFACE_1_IDENT, PNET_DIR_NO_IO, 0, 0);
  plug_submodule(PNET_SLOT_DAP_IDENT, PNET_SUBSLOT_DAP_INTERFACE_1_PORT_0_IDENT,
                 g_station_info.dap_mod_ident,
                 PNET_SUBMOD_DAP_INTERFACE_1_PORT_0_IDENT, PNET_DIR_NO_IO, 0,
                 0);
}

bool pnet_if::periodic(
#ifdef TIBERIUS_CAN
    can_if &can
#endif
) {
  pnet_handle_periodic(m_pnet);
  if (m_needs_app_ready) {
    m_needs_app_ready = false;
    pnet_application_ready(m_pnet, m_main_arep);
  }

#ifdef TIBERIUS_CAN
  for (uint16_t slot = 0; slot < g_station_info.dev_max_slot; ++slot) {
    if ((m_slot_bitmask & (1u << slot)) != 0) {
      bool is_updated = false;
      can_output_state state{};
      uint16_t output_length = sizeof(state);
      uint8_t output_iops;
      if (pnet_output_get_data_and_iops(m_pnet, 0, slot, 1, &is_updated,
                                        (uint8_t *)&state, &output_length,
                                        &output_iops) == 0 &&
          output_length >= sizeof(state)) {
        can.find_or_create_talon(slot).set_can_output_state(state);
      }
    }
  }
#endif

  return m_error;
}

#ifdef TIBERIUS_JS
void pnet_if::send_updates(const joystick_state &state) {
  pnet_input_set_data_and_iops(m_pnet, 0, 1, 1, (uint8_t *)&state,
                               sizeof(state), PNET_IOXS_GOOD);
}
#endif

#ifdef TIBERIUS_CAN
void pnet_if::send_updates(const talon_srx &talon) {
  assert(talon.m_dev_id <= g_station_info.dev_max_slot);
  can_input_state state = talon.get_can_input_state();
  pnet_input_set_data_and_iops(m_pnet, 0, talon.m_dev_id, 1, (uint8_t *)&state,
                               sizeof(state), PNET_IOXS_GOOD);
}
#endif

pnet_if::pnet_if(const char *netif, uint32_t tick_us) : m_cfg(this, netif) {
  m_pnet = pnet_init(netif, tick_us, &m_cfg);
  if (m_pnet)
    plug_dap();
}
