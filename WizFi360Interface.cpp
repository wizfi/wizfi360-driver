/* WizFi360 implementation of NetworkInterfaceAPI
 * Copyright (c) 2015 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if DEVICE_SERIAL && DEVICE_INTERRUPTIN && defined(MBED_CONF_EVENTS_PRESENT) && defined(MBED_CONF_NSAPI_PRESENT) && defined(MBED_CONF_RTOS_API_PRESENT)

#include <string.h>
#include <stdint.h>

#include "WizFi360.h"
#include "WizFi360Interface.h"
#include "events/EventQueue.h"
#include "events/mbed_shared_queues.h"
#include "netsocket/nsapi_types.h"
#include "mbed_trace.h"
#include "platform/Callback.h"
#include "platform/mbed_atomic.h"
#include "platform/mbed_debug.h"
#include "rtos/ThisThread.h"

using namespace std::chrono;

#ifndef MBED_CONF_WIZFI360_DEBUG
#define MBED_CONF_WIZFI360_DEBUG false
#endif

#ifndef MBED_CONF_WIZFI360_RTS
#define MBED_CONF_WIZFI360_RTS NC
#endif

#ifndef MBED_CONF_WIZFI360_CTS
#define MBED_CONF_WIZFI360_CTS NC
#endif

#ifndef MBED_CONF_WIZFI360_RST
#define MBED_CONF_WIZFI360_RST NC
#endif

#ifndef MBED_CONF_WIZFI360_PWR
#define MBED_CONF_WIZFI360_PWR NC
#endif

#define TRACE_GROUP  "WIZFII" // WizFi360 Interface

#define WIZFI360_WIFI_IF_NAME "es0"

#define LOCAL_ADDR "127.0.0.1"

using namespace mbed;
using namespace rtos;

#if defined MBED_CONF_WIZFI360_TX && defined MBED_CONF_WIZFI360_RX
WizFi360Interface::WizFi360Interface()
    : _wizfi(MBED_CONF_WIZFI360_TX, MBED_CONF_WIZFI360_RX, MBED_CONF_WIZFI360_DEBUG, MBED_CONF_WIZFI360_RTS, MBED_CONF_WIZFI360_CTS),
      _rst_pin(MBED_CONF_WIZFI360_RST), // Notice that Pin7 CH_EN cannot be left floating if used as reset
      _pwr_pin(MBED_CONF_WIZFI360_PWR),
      _ap_sec(NSAPI_SECURITY_UNKNOWN),
      _if_blocking(true),
#if MBED_CONF_RTOS_PRESENT
      _if_connected(_cmutex),
#endif
      _initialized(false),
      _connect_retval(NSAPI_ERROR_OK),
      _disconnect_retval(NSAPI_ERROR_OK),
      _conn_stat(NSAPI_STATUS_DISCONNECTED),
      _conn_stat_cb(),
      _global_event_queue(mbed_event_queue()), // Needs to be set before attaching event() to SIGIO
      _oob_event_id(0),
      _connect_event_id(0),
      _disconnect_event_id(0),
      _software_conn_stat(IFACE_STATUS_DISCONNECTED),
      _dhcp(true)
{
    memset(_cbs, 0, sizeof(_cbs));
    memset(ap_ssid, 0, sizeof(ap_ssid));
    memset(ap_pass, 0, sizeof(ap_pass));

    _ch_info.track_ap = true;
    strncpy(_ch_info.country_code, MBED_CONF_WIZFI360_COUNTRY_CODE, sizeof(_ch_info.country_code));
    _ch_info.channel_start = MBED_CONF_WIZFI360_CHANNEL_START;
    _ch_info.channels = MBED_CONF_WIZFI360_CHANNELS;

    _wizfi.sigio(this, &WizFi360Interface::event);
    _wizfi.set_timeout();
    _wizfi.attach(this, &WizFi360Interface::refresh_conn_state_cb);

    for (int i = 0; i < WIZFI360_SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].sport = 0;
    }
    _wizfi.uart_enable_input(false);
}
#endif

// WizFi360Interface implementation
WizFi360Interface::WizFi360Interface(PinName tx, PinName rx, bool debug, PinName rts, PinName cts, PinName rst, PinName pwr)
    : _wizfi(tx, rx, debug, rts, cts),
      _rst_pin(rst),
      _pwr_pin(pwr),
      _ap_sec(NSAPI_SECURITY_UNKNOWN),
      _if_blocking(true),
#if MBED_CONF_RTOS_PRESENT
      _if_connected(_cmutex),
#endif
      _initialized(false),
      _connect_retval(NSAPI_ERROR_OK),
      _disconnect_retval(NSAPI_ERROR_OK),
      _conn_stat(NSAPI_STATUS_DISCONNECTED),
      _conn_stat_cb(),
      _global_event_queue(mbed_event_queue()), // Needs to be set before attaching event() to SIGIO
      _oob_event_id(0),
      _connect_event_id(0),
      _disconnect_event_id(0),
      _software_conn_stat(IFACE_STATUS_DISCONNECTED),
      _dhcp(true)
{
    memset(_cbs, 0, sizeof(_cbs));
    memset(ap_ssid, 0, sizeof(ap_ssid));
    memset(ap_pass, 0, sizeof(ap_pass));

    _ch_info.track_ap = true;
    strncpy(_ch_info.country_code, MBED_CONF_WIZFI360_COUNTRY_CODE, sizeof(_ch_info.country_code));
    _ch_info.channel_start = MBED_CONF_WIZFI360_CHANNEL_START;
    _ch_info.channels = MBED_CONF_WIZFI360_CHANNELS;

    _wizfi.sigio(this, &WizFi360Interface::event);
    _wizfi.set_timeout();
    _wizfi.attach(this, &WizFi360Interface::refresh_conn_state_cb);

    for (int i = 0; i < WIZFI360_SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].sport = 0;
    }
    _wizfi.uart_enable_input(false);
}

WizFi360Interface::~WizFi360Interface()
{
    if (_oob_event_id) {
        _global_event_queue->cancel(_oob_event_id);
    }

    _cmutex.lock();
    if (_connect_event_id) {
        _global_event_queue->cancel(_connect_event_id);
    }
    _cmutex.unlock();

    // Power down the modem
    _rst_pin.rst_assert();
    // Power off the modem
    _pwr_pin.power_off();
}

WizFi360Interface::ResetPin::ResetPin(PinName rst_pin) : _rst_pin(mbed::DigitalOut(rst_pin, 1))
{
}

void WizFi360Interface::ResetPin::rst_assert()
{
    if (_rst_pin.is_connected()) {
        _rst_pin = 0;
        tr_debug("rst_assert(): HW reset asserted.");
    }
}

void WizFi360Interface::ResetPin::rst_deassert()
{
    if (_rst_pin.is_connected()) {
        // Notice that Pin7 CH_EN cannot be left floating if used as reset
        _rst_pin = 1;
        tr_debug("rst_deassert(): HW reset deasserted.");
    }
}

bool WizFi360Interface::ResetPin::is_connected()
{
    return _rst_pin.is_connected();
}

WizFi360Interface::PowerPin::PowerPin(PinName pwr_pin) : _pwr_pin(mbed::DigitalOut(pwr_pin, !MBED_CONF_WIZFI360_POWER_ON_POLARITY))
{
}

void WizFi360Interface::PowerPin::power_on()
{
    if (_pwr_pin.is_connected()) {
        _pwr_pin = MBED_CONF_WIZFI360_POWER_ON_POLARITY;
        tr_debug("power_on(): HW power-on.");
        ThisThread::sleep_for(milliseconds(MBED_CONF_WIZFI360_POWER_ON_TIME_MS));
    }
}

void WizFi360Interface::PowerPin::power_off()
{
    if (_pwr_pin.is_connected()) {
        _pwr_pin = !MBED_CONF_WIZFI360_POWER_ON_POLARITY;
        tr_debug("power_off(): HW power-off.");
        ThisThread::sleep_for(milliseconds(MBED_CONF_WIZFI360_POWER_OFF_TIME_MS));
    }
}

void WizFi360Interface::_power_off()
{
    _rst_pin.rst_assert();
    _pwr_pin.power_off();
}

bool WizFi360Interface::PowerPin::is_connected()
{
    return _pwr_pin.is_connected();
}

int WizFi360Interface::connect(const char *ssid, const char *pass, nsapi_security_t security,
                              uint8_t channel)
{
    if (channel != 0) {
        return NSAPI_ERROR_UNSUPPORTED;
    }

    int err = set_credentials(ssid, pass, security);
    if (err) {
        return err;
    }

    return connect();
}

void WizFi360Interface::_connect_async()
{
    nsapi_error_t status = _init();
    if (status != NSAPI_ERROR_OK) {
        _connect_retval = status;
        _wizfi.uart_enable_input(false);
        _software_conn_stat = IFACE_STATUS_DISCONNECTED;
        //_conn_stat_cb will be called from refresh_conn_state_cb
        return;
    }

    if (_dhcp && !_wizfi.dhcp(true, 1)) {
        _connect_retval = NSAPI_ERROR_DHCP_FAILURE;
        _wizfi.uart_enable_input(false);
        _software_conn_stat = IFACE_STATUS_DISCONNECTED;
        //_conn_stat_cb will be called from refresh_conn_state_cb
        return;
    }
    _cmutex.lock();
    if (!_connect_event_id) {
        tr_debug("_connect_async(): Cancelled.");
        _cmutex.unlock();
        return;
    }
    _connect_retval = _wizfi.connect(ap_ssid, ap_pass);
    auto timepassed = _conn_timer.elapsed_time();
    if (_connect_retval == NSAPI_ERROR_OK
            || _connect_retval == NSAPI_ERROR_AUTH_FAILURE
            || _connect_retval == NSAPI_ERROR_NO_SSID
            || ((_if_blocking == true) && (timepassed >= WIZFI360_INTERFACE_CONNECT_TIMEOUT))) {
        _connect_event_id = 0;
        _conn_timer.stop();
        if (timepassed >= WIZFI360_INTERFACE_CONNECT_TIMEOUT && _connect_retval != NSAPI_ERROR_OK) {
            _connect_retval = NSAPI_ERROR_CONNECTION_TIMEOUT;
        }
        if (_connect_retval != NSAPI_ERROR_OK) {
            _wizfi.uart_enable_input(false);
            _software_conn_stat = IFACE_STATUS_DISCONNECTED;
        }
#if MBED_CONF_RTOS_PRESENT
        _if_connected.notify_all();
#endif
    } else {
        // Postpone to give other stuff time to run
        _connect_event_id = _global_event_queue->call_in(WIZFI360_INTERFACE_CONNECT_INTERVAL,
                                                         callback(this, &WizFi360Interface::_connect_async));
        if (!_connect_event_id) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                       "WizFi360Interface::_connect_async(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
        }
    }
    _cmutex.unlock();

    if (_connect_event_id == 0) {
        if (_conn_stat_cb) {
            _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
        }
        if (_conn_stat == NSAPI_STATUS_GLOBAL_UP || _conn_stat == NSAPI_STATUS_LOCAL_UP) {
            _software_conn_stat = IFACE_STATUS_CONNECTED;
        }
    }
}

int WizFi360Interface::connect()
{
    if (_software_conn_stat == IFACE_STATUS_CONNECTING) {
        return NSAPI_ERROR_BUSY;
    }
    if (_software_conn_stat == IFACE_STATUS_CONNECTED) {
        return NSAPI_ERROR_IS_CONNECTED;
    }

    if (strlen(ap_ssid) == 0) {
        return NSAPI_ERROR_NO_SSID;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {
        if (strlen(ap_pass) < WIZFI360_PASSPHRASE_MIN_LENGTH) {
            return NSAPI_ERROR_PARAMETER;
        }
    }
    if (!_if_blocking) {
        bool ret = _cmutex.trylock();
        if (ret == false) {
            return NSAPI_ERROR_BUSY;
        }
    } else {
        _cmutex.lock();
    }
    _software_conn_stat = IFACE_STATUS_CONNECTING;
    _wizfi.uart_enable_input(true);
    _connect_retval = NSAPI_ERROR_NO_CONNECTION;
    MBED_ASSERT(!_connect_event_id);
    _conn_timer.stop();
    _conn_timer.reset();
    _conn_timer.start();
    _connect_event_id = _global_event_queue->call(callback(this, &WizFi360Interface::_connect_async));

    if (!_connect_event_id) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                   "connect(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
    }

#if MBED_CONF_RTOS_PRESENT
    while (_if_blocking && (_conn_status_to_error() != NSAPI_ERROR_IS_CONNECTED)
            && (_connect_retval == NSAPI_ERROR_NO_CONNECTION)) {
        _if_connected.wait();
    }
#endif

    _cmutex.unlock();

    if (!_if_blocking) {
        return NSAPI_ERROR_OK;
    } else {
        return _connect_retval;
    }
}

int WizFi360Interface::set_credentials(const char *ssid, const char *pass, nsapi_security_t security)
{
    nsapi_error_t status = _conn_status_to_error();
    if (_software_conn_stat == IFACE_STATUS_CONNECTING) {
        return NSAPI_ERROR_BUSY;
    }
    if (status != NSAPI_ERROR_NO_CONNECTION) {
        return status;
    }

    _ap_sec = security;

    if (!ssid) {
        return NSAPI_ERROR_PARAMETER;
    }

    int ssid_length = strlen(ssid);

    if (ssid_length > 0
            && ssid_length <= WIZFI360_SSID_MAX_LENGTH) {
        memset(ap_ssid, 0, sizeof(ap_ssid));
        strncpy(ap_ssid, ssid, WIZFI360_SSID_MAX_LENGTH);
    } else {
        return NSAPI_ERROR_PARAMETER;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {

        if (!pass) {
            return NSAPI_ERROR_PARAMETER;
        }

        int pass_length = strlen(pass);
        if (pass_length >= WIZFI360_PASSPHRASE_MIN_LENGTH
                && pass_length <= WIZFI360_PASSPHRASE_MAX_LENGTH) {
            memset(ap_pass, 0, sizeof(ap_pass));
            strncpy(ap_pass, pass, WIZFI360_PASSPHRASE_MAX_LENGTH);
        } else {
            return NSAPI_ERROR_PARAMETER;
        }
    } else {
        memset(ap_pass, 0, sizeof(ap_pass));
    }

    return NSAPI_ERROR_OK;
}

int WizFi360Interface::set_channel(uint8_t channel)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

nsapi_error_t WizFi360Interface::set_network(const SocketAddress &ip_address, const SocketAddress &netmask, const SocketAddress &gateway)
{
    nsapi_error_t init_result = _init();
    if (NSAPI_ERROR_OK != init_result) {
        return init_result;
    }

    // netmask and gateway switched on purpose. WizFi takes different argument order.
    if (_wizfi.set_ip_addr(ip_address.get_ip_address(), gateway.get_ip_address(), netmask.get_ip_address())) {
        _dhcp = false;
        return NSAPI_ERROR_OK;
    } else {
        return NSAPI_ERROR_DEVICE_ERROR;
    }
}

nsapi_error_t WizFi360Interface::set_dhcp(bool dhcp)
{
    nsapi_error_t init_result = _init();
    if (NSAPI_ERROR_OK != init_result) {
        return init_result;
    }

    _dhcp = dhcp;
    if (_wizfi.dhcp(dhcp, 1)) {
        return NSAPI_ERROR_OK;
    } else {
        return NSAPI_ERROR_DEVICE_ERROR;
    }
}

void WizFi360Interface::_disconnect_async()
{
    _cmutex.lock();
    _disconnect_retval = _wizfi.disconnect() ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
    auto timepassed = _conn_timer.elapsed_time();

    if (_disconnect_retval == NSAPI_ERROR_OK || ((_if_blocking == true) && (timepassed >= WIZFI360_INTERFACE_CONNECT_TIMEOUT))) {

        if (timepassed >= WIZFI360_INTERFACE_CONNECT_TIMEOUT && _connect_retval != NSAPI_ERROR_OK) {
            _disconnect_retval = NSAPI_ERROR_CONNECTION_TIMEOUT;
        } else {
            if (_conn_stat != NSAPI_STATUS_DISCONNECTED) {
                _conn_stat = NSAPI_STATUS_DISCONNECTED;
            }
            // In case the status update arrives later inform upper layers manually
            _disconnect_event_id = 0;
            _conn_timer.stop();
            _connect_retval = NSAPI_ERROR_NO_CONNECTION;
        }

        _power_off();
        _software_conn_stat = IFACE_STATUS_DISCONNECTED;
#if MBED_CONF_RTOS_PRESENT
        _if_connected.notify_all();
#endif

    } else {
        // Postpone to give other stuff time to run
        _disconnect_event_id = _global_event_queue->call_in(
                                   WIZFI360_INTERFACE_CONNECT_INTERVAL,
                                   callback(this, &WizFi360Interface::_disconnect_async));
        if (!_disconnect_event_id) {
            MBED_ERROR(
                MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                "WizFi360Interface::_disconnect_async(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
        }
    }
    _cmutex.unlock();

    _wizfi.uart_enable_input(false);
    if (_disconnect_event_id == 0) {
        if (_conn_stat_cb) {
            _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
        }
    }
}

int WizFi360Interface::disconnect()
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTING) {
        return NSAPI_ERROR_BUSY;
    }
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        return NSAPI_ERROR_NO_CONNECTION;
    }
    if (!_if_blocking) {
        bool ret = _cmutex.trylock();
        if (ret == false) {
            return NSAPI_ERROR_BUSY;
        }
    } else {
        _cmutex.lock();
    }
    if (_connect_event_id) {
        _global_event_queue->cancel(_connect_event_id);
        _connect_event_id = 0; // cancel asynchronous connection attempt if one is ongoing
    }
    _software_conn_stat = IFACE_STATUS_DISCONNECTING;

    _disconnect_retval = NSAPI_ERROR_IS_CONNECTED;
    _disconnect_event_id = 0;

    _initialized = false;
    _conn_timer.stop();
    _conn_timer.reset();
    _conn_timer.start();

    _disconnect_event_id = _global_event_queue->call(
                               callback(this, &WizFi360Interface::_disconnect_async));

    if (!_disconnect_event_id) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM),
                   "disconnect(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
    }

#if MBED_CONF_RTOS_PRESENT
    while (_if_blocking
            && (_conn_status_to_error() != NSAPI_ERROR_NO_CONNECTION)
            && (_disconnect_retval != NSAPI_ERROR_OK)) {
        _if_connected.wait();
    }
#endif

    _cmutex.unlock();
    if (!_if_blocking) {
        return NSAPI_ERROR_OK;
    } else {
        return _disconnect_retval;
    }
}

nsapi_error_t WizFi360Interface::get_ip_address(SocketAddress *address)
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(true);
    }

    const char *ip_buff = _wizfi.ip_addr();
    if (!ip_buff || strcmp(ip_buff, "0.0.0.0") == 0) {
        ip_buff = NULL;
    }
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(false);
    }
    if (ip_buff) {
        address->set_ip_address(ip_buff);
        return NSAPI_ERROR_OK;
    }
    return NSAPI_ERROR_NO_ADDRESS;
}

const char *WizFi360Interface::get_mac_address()
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(true);
    }
    const char *ret = _wizfi.mac_addr();

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(false);
    }
    return ret;
}

nsapi_error_t WizFi360Interface::get_gateway(SocketAddress *address)
{
    if (address == nullptr) {
        return NSAPI_ERROR_PARAMETER;
    }
    if (_conn_stat == NSAPI_STATUS_DISCONNECTED) {
        return NSAPI_ERROR_NO_CONNECTION;
    }

    if (!address->set_ip_address(_wizfi.gateway())) {
        return NSAPI_ERROR_NO_ADDRESS;
    }

    return NSAPI_ERROR_OK;
}

const char *WizFi360Interface::get_gateway()
{
    return _conn_stat != NSAPI_STATUS_DISCONNECTED ? _wizfi.gateway() : NULL;
}

nsapi_error_t WizFi360Interface::get_netmask(SocketAddress *address)
{
    if (address == nullptr) {
        return NSAPI_ERROR_PARAMETER;
    }
    if (_conn_stat == NSAPI_STATUS_DISCONNECTED) {
        return NSAPI_ERROR_NO_CONNECTION;
    }

    if (!address->set_ip_address(_wizfi.netmask())) {
        return NSAPI_ERROR_NO_ADDRESS;
    }

    return NSAPI_ERROR_OK;
}

const char *WizFi360Interface::get_netmask()
{
    return _conn_stat != NSAPI_STATUS_DISCONNECTED ? _wizfi.netmask() : NULL;
}

nsapi_error_t WizFi360Interface::get_time(std::tm *t)
{
    _init();
    return _wizfi.get_sntp_time(t) ? NSAPI_ERROR_OK : NSAPI_ERROR_TIMEOUT;
}

char *WizFi360Interface::get_interface_name(char *interface_name)
{
    memcpy(interface_name, WIZFI360_WIFI_IF_NAME, sizeof(WIZFI360_WIFI_IF_NAME));
    return interface_name;
}

int8_t WizFi360Interface::get_rssi()
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(true);
    }

    int8_t ret = _wizfi.rssi();

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(false);
    }

    return ret;
}

int WizFi360Interface::scan(WiFiAccessPoint *res, unsigned count)
{
    return scan(res, count, SCANMODE_ACTIVE);
}

int WizFi360Interface::scan(WiFiAccessPoint *res, unsigned count, scan_mode mode, mbed::chrono::milliseconds_u32 t_max, mbed::chrono::milliseconds_u32 t_min)
{
    if (t_max > WIZFI360_SCAN_TIME_MAX) {
        return NSAPI_ERROR_PARAMETER;
    }
    if (mode == SCANMODE_ACTIVE && t_min > t_max) {
        return NSAPI_ERROR_PARAMETER;
    }

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(true);
    }

    nsapi_error_t status = _init();
    if (status != NSAPI_ERROR_OK) {
        if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
            _wizfi.uart_enable_input(false);
        }
        return status;
    }

    int ret = _wizfi.scan(res, count, (mode == SCANMODE_ACTIVE ? WizFi360::SCANMODE_ACTIVE : WizFi360::SCANMODE_PASSIVE),
                        t_max, t_min);

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _wizfi.uart_enable_input(false);
    }
    return ret;
}

#if MBED_CONF_WIZFI360_BUILT_IN_DNS
nsapi_error_t WizFi360Interface::gethostbyname(const char *name, SocketAddress *address, nsapi_version_t version, const char *interface_name)
{
    char ip[NSAPI_IPv4_SIZE];
    memset(ip, 0, NSAPI_IPv4_SIZE);
    if (!_wizfi.dns_lookup(name, ip)) {
        return NSAPI_ERROR_DNS_FAILURE;
    }
    if (!address->set_ip_address(ip)) {
        return NSAPI_ERROR_DNS_FAILURE;
    }

    return NSAPI_ERROR_OK;
}


nsapi_error_t WizFi360Interface::add_dns_server(const SocketAddress &address, const char *interface_name)
{
    return NSAPI_ERROR_OK;
}
#endif

bool WizFi360Interface::_get_firmware_ok()
{
    WizFi360::fw_at_version at_v = _wizfi.at_version();
    if (at_v.major < WIZFI360_AT_VERSION_MAJOR) {
        debug("WizFi360: ERROR: AT Firmware v%d incompatible with this driver.", at_v.major);
        debug("Update at least to v%d - https://github.com/wizfi/Release/tree/master/Binary\n", WIZFI360_AT_VERSION_MAJOR);
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_UNSUPPORTED), "Too old AT firmware");
    }
    WizFi360::fw_sdk_version sdk_v = _wizfi.sdk_version();
    if (sdk_v.major < WIZFI360_SDK_VERSION_MAJOR) {
        debug("WizFi360: ERROR: Firmware v%d incompatible with this driver.", sdk_v.major);
        debug("Update at least to v%d - https://github.com/wizfi/Release/tree/master/Binary\n", WIZFI360_SDK_VERSION_MAJOR);
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_UNSUPPORTED), "Too old SDK firmware");
    }

    return true;
}

nsapi_error_t WizFi360Interface::_init(void)
{
    if (!_initialized) {
        _pwr_pin.power_off();
        _pwr_pin.power_on();

        if (_reset() != NSAPI_ERROR_OK) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.echo_off()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.start_uart_hw_flow_ctrl()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_get_firmware_ok()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.set_default_wifi_mode(WizFi360::WIFIMODE_STATION)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.set_country_code_policy(true, _ch_info.country_code, _ch_info.channel_start, _ch_info.channels)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.cond_enable_tcp_passive_mode()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.startup(WizFi360::WIFIMODE_STATION)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
#if MBED_CONF_WIZFI360_SNTP_ENABLE
        if (!_wizfi.set_sntp_config(MBED_CONF_WIZFI360_SNTP_ENABLE,
                                  MBED_CONF_WIZFI360_SNTP_TIMEZONE,
                                  MBED_CONF_WIZFI360_SNTP_SERVER0,
                                  MBED_CONF_WIZFI360_SNTP_SERVER1,
                                  MBED_CONF_WIZFI360_SNTP_SERVER2)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
#endif
        _initialized = true;
    }
    return NSAPI_ERROR_OK;
}

nsapi_error_t WizFi360Interface::_reset()
{
    if (_rst_pin.is_connected()) {
        _rst_pin.rst_assert();
        // If you happen to use Pin7 CH_EN as reset pin, not needed otherwise
        // http://wizwiki.net/wiki/lib/exe/fetch.php/products:wizfi360:wizfi360ds:wizfi360_hardware_design_guide_v103_en.pdf
        // First need to round up when converting to kernel ticks (eg 200us -> 1ms).
        auto delay = duration_cast<Kernel::Clock::duration_u32>(200us);
        if (delay < 200us) {
            delay++;
        }
        // Then need to round the clock-resolution duration up; if we were at the end of a tick
        // period, it might flip immediately.
        delay++;
        ThisThread::sleep_for(delay);
        _wizfi.flush();
        _rst_pin.rst_deassert();
    } else {
        _wizfi.flush();
        if (!_wizfi.at_available()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_wizfi.reset()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
    }

    return _wizfi.at_available() ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
}

int WizFi360Interface::socket_open(void **handle, nsapi_protocol_t proto)
{
    // Look for an unused socket
    int id = -1;

    for (int i = 0; i < WIZFI360_SOCKET_COUNT; i++) {
        if (!_sock_i[i].open) {
            id = i;
            _sock_i[i].open = true;
            break;
        }
    }

    if (id == -1) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    struct wizfi360_socket *socket = new struct wizfi360_socket;
    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    socket->id = id;
    socket->proto = proto;
    socket->connected = false;
    socket->bound = false;
    socket->keepalive = 0;
    *handle = socket;
    return 0;
}

int WizFi360Interface::socket_close(void *handle)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;
    int err = 0;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->connected && !_wizfi.close(socket->id)) {
        err = NSAPI_ERROR_DEVICE_ERROR;
    }

    if (socket->bound && !_wizfi.close(socket->id)) {
        err = NSAPI_ERROR_DEVICE_ERROR;
    }

    _cbs[socket->id].callback = NULL;
    _cbs[socket->id].data = NULL;
    core_util_atomic_store_u8(&_cbs[socket->id].deferred, false);

    socket->connected = false;
    socket->bound = false;
    _sock_i[socket->id].open = false;
    _sock_i[socket->id].sport = 0;
    delete socket;
    return err;
}

int WizFi360Interface::socket_bind(void *handle, const SocketAddress &address)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->proto == NSAPI_UDP) {
        if (address.get_addr().version != NSAPI_UNSPEC) {
            return NSAPI_ERROR_UNSUPPORTED;
        }

        for (int id = 0; id < WIZFI360_SOCKET_COUNT; id++) {
            if (_sock_i[id].sport == address.get_port() && id != socket->id) { // Port already reserved by another socket
                return NSAPI_ERROR_PARAMETER;
            } else if (id == socket->id && (socket->connected || socket->bound)) {
                return NSAPI_ERROR_PARAMETER;
            }
        }
        _sock_i[socket->id].sport = address.get_port();

        int ret = _wizfi.open_udp(socket->id, LOCAL_ADDR, address.get_port(), _sock_i[socket->id].sport, 2);

        socket->bound = (ret == NSAPI_ERROR_OK) ? true : false;

        return ret;
    }

    return NSAPI_ERROR_UNSUPPORTED;
}

int WizFi360Interface::socket_listen(void *handle, int backlog)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

int WizFi360Interface::socket_connect(void *handle, const SocketAddress &addr)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;
    nsapi_error_t ret;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->proto == NSAPI_UDP) {
        ret = _wizfi.open_udp(socket->id, addr.get_ip_address(), addr.get_port(), _sock_i[socket->id].sport, 0);
    } else {
        ret = _wizfi.open_tcp(socket->id, addr.get_ip_address(), addr.get_port(), socket->keepalive);
    }

    socket->connected = (ret == NSAPI_ERROR_OK) ? true : false;

    return ret;
}

int WizFi360Interface::socket_accept(void *server, void **socket, SocketAddress *addr)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

int WizFi360Interface::socket_send(void *handle, const void *data, unsigned size)
{
    nsapi_size_or_error_t status;
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;
    uint8_t expect_false = false;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!_sock_i[socket->id].open) {
        return NSAPI_ERROR_CONNECTION_LOST;
    }

    if (!size) {
        // Firmware limitation
        return socket->proto == NSAPI_TCP ? 0 : NSAPI_ERROR_UNSUPPORTED;
    }

    status = _wizfi.send(socket->id, data, size);

    if (status == NSAPI_ERROR_WOULD_BLOCK
            && socket->proto == NSAPI_TCP
            && core_util_atomic_cas_u8(&_cbs[socket->id].deferred, &expect_false, true)) {
        tr_debug("socket_send(...): Postponing SIGIO from the device.");
        if (!_global_event_queue->call_in(50ms, callback(this, &WizFi360Interface::event_deferred))) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                       "socket_send(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
        }

    } else if (status == NSAPI_ERROR_WOULD_BLOCK && socket->proto == NSAPI_UDP) {
        status = NSAPI_ERROR_DEVICE_ERROR;
    }

    return status;
}

int WizFi360Interface::socket_recv(void *handle, void *data, unsigned size)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!_sock_i[socket->id].open) {
        return NSAPI_ERROR_CONNECTION_LOST;
    }

    int32_t recv;
    if (socket->proto == NSAPI_TCP) {
        recv = _wizfi.recv_tcp(socket->id, data, size);
        if (recv <= 0 && recv != NSAPI_ERROR_WOULD_BLOCK) {
            socket->connected = false;
        }
    } else {
        recv = _wizfi.recv_udp(socket, data, size);
    }

    return recv;
}

int WizFi360Interface::socket_sendto(void *handle, const SocketAddress &addr, const void *data, unsigned size)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if ((strcmp(addr.get_ip_address(), "0.0.0.0") == 0) || !addr.get_port())  {
        return NSAPI_ERROR_DNS_FAILURE;
    }

    if (socket->connected && socket->addr != addr) {
        if (!_wizfi.close(socket->id)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        socket->connected = false;
    }

    if (!socket->connected && !socket->bound) {
        int err = socket_connect(socket, addr);
        if (err < 0) {
            return err;
        }
        socket->addr = addr;
    }

    if (socket->bound) {
        socket->addr = addr;
    }

    return socket_send(socket, data, size);
}

int WizFi360Interface::socket_recvfrom(void *handle, SocketAddress *addr, void *data, unsigned size)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    int ret = socket_recv(socket, data, size);
    if (ret >= 0 && addr) {
        *addr = socket->addr;
    }

    return ret;
}

void WizFi360Interface::socket_attach(void *handle, void (*callback)(void *), void *data)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;
    _cbs[socket->id].callback = callback;
    _cbs[socket->id].data = data;
}

nsapi_error_t WizFi360Interface::setsockopt(nsapi_socket_t handle, int level,
                                           int optname, const void *optval, unsigned optlen)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;

    if (!optlen) {
        return NSAPI_ERROR_PARAMETER;
    } else if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (level == NSAPI_SOCKET && socket->proto == NSAPI_TCP) {
        switch (optname) {
            case NSAPI_KEEPALIVE: {
                if (socket->connected) { // WizFi360 limitation, keepalive needs to be given before connecting
                    return NSAPI_ERROR_UNSUPPORTED;
                }

                if (optlen == sizeof(int)) {
                    int secs = *(int *)optval;
                    if (secs  >= 0 && secs <= 7200) {
                        socket->keepalive = secs;
                        return NSAPI_ERROR_OK;
                    }
                }
                return NSAPI_ERROR_PARAMETER;
            }
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;
}

nsapi_error_t WizFi360Interface::getsockopt(nsapi_socket_t handle, int level, int optname, void *optval, unsigned *optlen)
{
    struct wizfi360_socket *socket = (struct wizfi360_socket *)handle;

    if (!optval || !optlen) {
        return NSAPI_ERROR_PARAMETER;
    } else if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (level == NSAPI_SOCKET && socket->proto == NSAPI_TCP) {
        switch (optname) {
            case NSAPI_KEEPALIVE: {
                if (*optlen > sizeof(int)) {
                    *optlen = sizeof(int);
                }
                memcpy(optval, &(socket->keepalive), *optlen);
                return NSAPI_ERROR_OK;
            }
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;
}


void WizFi360Interface::event()
{
    if (!_oob_event_id) {
        // Throttles event creation by using arbitrary small delay
        _oob_event_id = _global_event_queue->call_in(50ms, callback(this, &WizFi360Interface::proc_oob_evnt));
        if (!_oob_event_id) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                       "WizFi360Interface::event(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
        }
    }

    for (int i = 0; i < WIZFI360_SOCKET_COUNT; i++) {
        if (_cbs[i].callback) {
            _cbs[i].callback(_cbs[i].data);
        }
    }
}

void WizFi360Interface::event_deferred()
{
    for (int i = 0; i < WIZFI360_SOCKET_COUNT; i++) {
        uint8_t expect_true = true;
        if (core_util_atomic_cas_u8(&_cbs[i].deferred, &expect_true, false) && _cbs[i].callback) {
            _cbs[i].callback(_cbs[i].data);
        }
    }
}

void WizFi360Interface::attach(Callback<void(nsapi_event_t, intptr_t)> status_cb)
{
    _conn_stat_cb = status_cb;
}

nsapi_connection_status_t WizFi360Interface::get_connection_status() const
{
    return _conn_stat;
}

#if MBED_CONF_WIZFI360_PROVIDE_DEFAULT

WiFiInterface *WiFiInterface::get_default_instance()
{
    static WizFi360Interface wizfi;
    return &wizfi;
}

#endif

void WizFi360Interface::refresh_conn_state_cb()
{
    nsapi_connection_status_t prev_stat = _conn_stat;
    _conn_stat = _wizfi.connection_status();

    switch (_conn_stat) {
        // Doesn't require changes
        case NSAPI_STATUS_CONNECTING:
        case NSAPI_STATUS_GLOBAL_UP:
            if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
                _software_conn_stat = IFACE_STATUS_CONNECTED;
            }
            break;
        // Start from scratch if connection drops/is dropped
        case NSAPI_STATUS_DISCONNECTED:
            if (_software_conn_stat == IFACE_STATUS_CONNECTED) {
                _software_conn_stat = IFACE_STATUS_DISCONNECTED;
            }
            break;
        // Handled on AT layer
        case NSAPI_STATUS_LOCAL_UP:
        case NSAPI_STATUS_ERROR_UNSUPPORTED:
        default:
            _initialized = false;
            _conn_stat = NSAPI_STATUS_DISCONNECTED;
            for (int i = 0; i < WIZFI360_SOCKET_COUNT; i++) {
                _sock_i[i].open = false;
                _sock_i[i].sport = 0;
            }
    }

    if (prev_stat == _conn_stat) {
        return;
    }

    tr_debug("refresh_conn_state_cb(): Changed to %d.", _conn_stat);

    if (_conn_stat_cb) {
        // _conn_stat_cb will be called in _connect_async or disconnect_assync to avoid race condition
        if ((_software_conn_stat == IFACE_STATUS_CONNECTING
                || _software_conn_stat == IFACE_STATUS_DISCONNECTING)
                && (_conn_stat != NSAPI_STATUS_CONNECTING)) {
            return;
        }

        _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
    }
}

void WizFi360Interface::proc_oob_evnt()
{
    _oob_event_id = 0; // Allows creation of a new event
    _wizfi.bg_process_oob(WIZFI360_RECV_TIMEOUT, true);
}

nsapi_error_t WizFi360Interface::_conn_status_to_error()
{
    nsapi_error_t ret;

    switch (_conn_stat) {
        case NSAPI_STATUS_DISCONNECTED:
            ret = NSAPI_ERROR_NO_CONNECTION;
            break;
        case NSAPI_STATUS_CONNECTING:
            ret = NSAPI_ERROR_ALREADY;
            break;
        case NSAPI_STATUS_GLOBAL_UP:
            ret = NSAPI_ERROR_IS_CONNECTED;
            break;
        default:
            ret = NSAPI_ERROR_DEVICE_ERROR;
    }

    return ret;
}

nsapi_error_t WizFi360Interface::set_blocking(bool blocking)
{
    _if_blocking = blocking;

    return NSAPI_ERROR_OK;
}

nsapi_error_t WizFi360Interface::set_country_code(bool track_ap, const char *country_code, int len, int channel_start, int channels)
{
    for (int i = 0; i < len; i++) {
        // Validation done by firmware
        if (!country_code[i]) {
            tr_warning("set_country_code(): Invalid country code.");
            return NSAPI_ERROR_PARAMETER;
        }
    }

    _ch_info.track_ap = track_ap;

    // Firmware takes only first three characters
    strncpy(_ch_info.country_code, country_code, sizeof(_ch_info.country_code));
    _ch_info.country_code[sizeof(_ch_info.country_code) - 1] = '\0';

    _ch_info.channel_start = channel_start;
    _ch_info.channels = channels;

    return NSAPI_ERROR_OK;
}

#endif
