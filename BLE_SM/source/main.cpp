/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "SecurityManager.h"

#include <cstdlib>

#include "commander.h"

// our serial interface cli class
Commander _cmd;
static events::EventQueue _event_queue;

/** This example demonstrates all the basic setup required
 *  for pairing and setting up link security both as a central and peripheral
 *
 *  The example is implemented as two classes, one for the peripheral and one
 *  for central inheriting from a common base. They are run in sequence and
 *  require a peer device to connect to. During the peripheral device demonstration
 *  a peer device is required to connect. In the central device demonstration
 *  this peer device will be scanned for and connected to - therefore it should
 *  be advertising with the same address as when it connected.
 *
 *  During the test output is written on the serial connection to monitor its
 *  progress.
 */

static const uint8_t DEVICE_NAME[] = "MRay_device";

/* for demonstration purposes we will store the peer device address
 * of the device that connects to us in the first demonstration
 * so we can use its address to reconnect to it later */
static BLEProtocol::AddressBytes_t _peer_address;

std::map<std::string, Gap::AdvertisementCallbackParams_t> _scan_map;

std::string convert_to_string(uint8_t* input)
{
    char buffer[17];
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
            input[5], input[4], input[3], input[2], input[1], input[0]);
    std::string output(buffer);
    return output;
}

void convert_to_byte_array(string input, uint8_t* output)
{
    size_t pos = 0;
    string token;
    int i = 5;
    char * p;
    long n;
    string delimiter = ":";

    while ((pos = input.find(delimiter)) != string::npos) {
        token = input.substr(0, pos);
        n = strtol(token.c_str(), &p, 16);
        output[i--] = n;
        input.erase(0, pos + delimiter.length());
    }

    for (i = 0; i < input.size(); i += 2) {
        token = input.substr(i, 2);
        output[i/2] = strtol(token.c_str(), &p, 16);
    }
}

/** Base class for both peripheral and central. The same class that provides
 *  the logic for the application also implements the SecurityManagerEventHandler
 *  which is the interface used by the Security Manager to communicate events
 *  back to the applications. You can provide overrides for a selection of events
 *  your application is interested in.
 */
class SMDevice : private mbed::NonCopyable<SMDevice>,
                 public SecurityManager::EventHandler
{
public:
    SMDevice() :
        _led1(LED1, 0),
        _handle(0),
        _master(false) { };

    virtual ~SMDevice()
    {
        if (BLE::Instance().hasInitialized()) {
            BLE::Instance().shutdown();
        }
    };

    /** Start BLE interface initialisation */
    void run()
    {
        ble_error_t error;

        /* to show we're running we'll blink every 500ms */
        _event_queue.call_every(500, this, &SMDevice::blink);

        if (BLE::Instance().hasInitialized()) {
            _cmd.printf("Ble instance already initialised.\r\n");
            return;
        }

        /* this will inform us off all events so we can schedule their handling
         * using our event queue */
        BLE::Instance().onEventsToProcess(
            makeFunctionPointer(this, &SMDevice::schedule_ble_events)
        );

        /* handle timeouts, for example when connection attempts fail */
        BLE::Instance().gap().onTimeout(
            makeFunctionPointer(this, &SMDevice::on_timeout)
        );

        error = BLE::Instance().init(this, &SMDevice::on_init_complete);

        if (error) {
            _cmd.printf("Error returned by BLE::init.\r\n");
            return;
        }

        /* this will not return until shutdown */
        _event_queue.dispatch_forever();
    };

    /* event handler functions */

    /** Indicate that the application needs to send secure connections OOB
     * data to the designated address. */
    virtual void oobGenerated(const ble::address_t *address,
                              const ble::oob_lesc_value_t *random,
                              const ble::oob_confirm_t *confirm)
    {
        _cmd.printf("Hash: ");
        for (int i = 0; i < 16; i++) {
            _cmd.printf("%02X", confirm->data()[i]);
        }
        _cmd.printf("\r\nRandom: ");
        for (int i = 0; i < 16; i++) {
            _cmd.printf("%02X", random->data()[i]);
        }
        _cmd.printf("\r\n");
    }

    /** Respond to a pairing request. This will be called by the stack
     * when a pairing request arrives and expects the application to
     * call acceptPairingRequest or cancelPairingRequest */
    virtual void pairingRequest(
        ble::connection_handle_t connectionHandle
    ) {
        _cmd.printf("Pairing requested - authorising\r\n");
        BLE::Instance().securityManager().acceptPairingRequest(connectionHandle);
    }

    /** Inform the application of a successful pairing. Terminate the demonstration. */
    virtual void pairingResult(
        ble::connection_handle_t connectionHandle,
        SecurityManager::SecurityCompletionStatus_t result
    ) {
        if (result == SecurityManager::SEC_STATUS_SUCCESS) {
            _cmd.printf("Pairing successful\r\n");
        } else {
            _cmd.printf("Pairing failed. Code: %d\r\n", result);
        }
    }

    /** Inform the application of change in encryption status. This will be
     * communicated through the serial port */
    virtual void linkEncryptionResult(
        ble::connection_handle_t connectionHandle,
        ble::link_encryption_t result
    ) {
        if (result == ble::link_encryption_t::ENCRYPTED) {
            _cmd.printf("Link ENCRYPTED\r\n");
        } else if (result == ble::link_encryption_t::ENCRYPTED_WITH_MITM) {
            _cmd.printf("Link ENCRYPTED_WITH_MITM\r\n");
        } else if (result == ble::link_encryption_t::NOT_ENCRYPTED) {
            _cmd.printf("Link NOT_ENCRYPTED\r\n");
        }
    }

    /** Look at scan payload to find a peer device and connect to it */
    void on_scan(const Gap::AdvertisementCallbackParams_t *params)
    {
        std::string mac = convert_to_string((uint8_t*)params->peerAddr);
        if (_scan_map.find(mac) == _scan_map.end()) {
            _cmd.printf("[NEW] Device %s RSSI is %d\r\n", mac.c_str(), params->rssi);
            _scan_map[mac] = *params;
        }
    };

private:

    virtual void start()
    {
        /* Set up and start advertising */

        ble_error_t error;
        GapAdvertisingData advertising_data;

        /* add advertising flags */
        advertising_data.addFlags(GapAdvertisingData::LE_GENERAL_DISCOVERABLE
                                  | GapAdvertisingData::BREDR_NOT_SUPPORTED);

        /* add device name */
        advertising_data.addData(
            GapAdvertisingData::COMPLETE_LOCAL_NAME,
            DEVICE_NAME,
            sizeof(DEVICE_NAME)
        );

        uint8_t oob = 1;
        uint8_t SM_OOB_FLAGS = 0x11;
        advertising_data.addData(
            (GapAdvertisingData::DataType_t)SM_OOB_FLAGS,
            &oob,
            sizeof(oob));

        error = BLE::Instance().gap().setAdvertisingPayload(advertising_data);

        if (error) {
            _cmd.printf("Error during Gap::setAdvertisingPayload\r\n");
            return;
        }

        /* advertise to everyone */
        BLE::Instance().gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
        /* how many milliseconds between advertisements, lower interval
         * increases the chances of being seen at the cost of more power */
        BLE::Instance().gap().setAdvertisingInterval(20);
        BLE::Instance().gap().setAdvertisingTimeout(0);

        BLE::Instance().securityManager().setPairingRequestAuthorisation(true);
    }

    /** This is called when BLE interface is initialised and starts the demonstration */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *event)
    {
        ble_error_t error;

        if (event->error) {
            _cmd.printf("Error during the initialisation\r\n");
            return;
        }

        /* This path will be used to store bonding information but will fallback
         * to storing in memory if file access fails (for example due to lack of a filesystem) */
        const char* db_path = "/fs/bt_sec_db";
        /* If the security manager is required this needs to be called before any
         * calls to the Security manager happen. */
        error = BLE::Instance().securityManager().init(
            true,
            false,
            SecurityManager::IO_CAPS_NONE,
            NULL,
            false,
            db_path
        );

        if (error) {
            _cmd.printf("Error during init %d\r\n", error);
            return;
        }

        error = BLE::Instance().securityManager().preserveBondingStateOnReset(true);

        if (error) {
            _cmd.printf("Error during preserveBondingStateOnReset %d\r\n", error);
        }

        /* Tell the security manager to use methods in this class to inform us
         * of any events. Class needs to implement SecurityManagerEventHandler. */
        BLE::Instance().securityManager().setSecurityManagerEventHandler(this);

        /* when scanning we want to connect to a peer device so we need to
         * attach callbacks that are used by Gap to notify us of events */
        BLE::Instance().gap().onConnection(this, &SMDevice::on_connect);
        BLE::Instance().gap().onDisconnection(this, &SMDevice::on_disconnect);

        /* start test in 500 ms */
        _event_queue.call_in(500, this, &SMDevice::start);
    };

    /** This is called by Gap to notify the application we connected,
     *  in our case it immediately request pairing */
    virtual void on_connect(const Gap::ConnectionCallbackParams_t *connection_event)
    {
        ble_error_t error;

        /* store the handle for future Security Manager requests */
        _handle = connection_event->handle;

        _cmd.printf("Connected\r\n");

        if (_master) {

            /* Request a change in link security. This will be done
             * indirectly by asking the master of the connection to
             * change it. Depending on circumstances different actions
             * may be taken by the master which will trigger events
             * which the applications should deal with. */
            error = BLE::Instance().securityManager().setLinkSecurity(
                _handle,
                SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM
            );

            if (error) {
                printf("Error during SM::setLinkSecurity %d\r\n", error);
                return;
            }
        }
    };

    /** This is called by Gap to notify the application we disconnected,
     *  in our case it ends the demonstration. */
    void on_disconnect(const Gap::DisconnectionCallbackParams_t *event)
    {
        _cmd.printf("Diconnected\r\n");
        _event_queue.break_dispatch();
        _master = false;
    };

    /** End demonstration unexpectedly. Called if timeout is reached during advertising,
     * scanning or connection initiation */
    void on_timeout(const Gap::TimeoutSource_t source)
    {
        _cmd.printf("Unexpected timeout - aborting\r\n");
        _event_queue.break_dispatch();
    };

    /** Schedule processing of events from the BLE in the event queue. */
    void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
    {
        _event_queue.call(mbed::callback(&context->ble, &BLE::processEvents));
    };

    /** Blink LED to show we're running */
    void blink(void)
    {
        _led1 = !_led1;
    };

private:
    DigitalOut _led1;

protected:
    ble::connection_handle_t _handle;

public:
    bool _master;
};

SMDevice conn_dev;

static void _cmd_cb_info(vector<string> &params)
{
    Gap::AddressType_t addr_type;
    Gap::Address_t addr;
    BLE::Instance().gap().getAddress(&addr_type, addr);
    _cmd.printf("Device MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    _cmd.printf("Device type: %d\r\n", addr_type);
}

static void _cmd_cb_ble_advertise(std::vector<string> &params)
{
    if (params.size() >= 2) {

        ble_error_t error;
        _cmd.printf("Turning advertising %s\r\n", params[1].c_str());
        if (params[1].compare("on") == 0) {
            error = BLE::Instance().gap().startAdvertising();
        } else {
            error = BLE::Instance().gap().stopAdvertising();
        }

        if (error) {
            _cmd.printf("Error during Gap::advertising %d\r\n", error);
            return;
        }
    }
}

static void _cmd_cb_ble_scan(std::vector<string> &params)
{
    if (params.size() >= 2) {

        ble_error_t error;
        if (params[1].compare("on") == 0) {
            error = BLE::Instance().gap().startScan(&conn_dev, &SMDevice::on_scan);
        } else {
            error = BLE::Instance().gap().stopScan();
        }

        if (error) {
            _cmd.printf("Error during Gap::scan %d\r\n", error);
            return;
        }
    }
}

static void _cmd_cb_ble_connect(std::vector<string> &params)
{
    if (params.size() >= 2 && _scan_map.find(params[1]) != _scan_map.end()) {

        conn_dev._master = true;
        _cmd.printf("Attempting connection with %s\r\n", params[1].c_str());
        ble_error_t error = BLE::Instance().gap().connect(
            _scan_map[params[1]].peerAddr,
            _scan_map[params[1]].peerAddrType,
            NULL,
            NULL);

        if (error) {
            _cmd.printf("Error during Gap::connect %d\r\n", error);
            return;
        }

    } else {
        _cmd.printf("Device not found. Please scan for your device first...\n");
    }
}

static void _cmd_cb_oob(vector<string> &params)
{
    if (params.size() >= 4) {
        uint8_t mac_address[6];
        uint8_t hash[16];
        uint8_t randomizer[16];
        convert_to_byte_array(params[1], hash);
        convert_to_byte_array(params[2], randomizer);
        convert_to_byte_array(params[3], mac_address);

        _cmd.printf("Remote OOB set for: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               mac_address[5], mac_address[4], mac_address[3],
               mac_address[2], mac_address[1], mac_address[0]);

        _cmd.printf("Hash: ");
        for (int i = 0; i<sizeof(hash); i++) {
            _cmd.printf("%02X", hash[i]);
        }
        _cmd.printf("\r\nRandomizer: ");
        for (int i = 0; i<sizeof(randomizer); i++) {
            _cmd.printf("%02X", randomizer[i]);
        }
        _cmd.printf("\r\n");

        ble_error_t error;
        error = BLE::Instance().securityManager().oobReceived((const ble::address_t*)mac_address,
                                                              (const ble::oob_lesc_value_t*)randomizer,
                                                              (const ble::oob_confirm_t*)hash);
        if (error) {
            _cmd.printf("OOB received error code: %u\r\n", error);
        }
    }
}

static void _cmd_cb_le_oob(std::vector<string> &params)
{
    Gap::AddressType_t addr_type;
    Gap::Address_t addr;
    BLE::Instance().gap().getAddress(&addr_type, addr);
    uint8_t converted_address[6];
    memcpy(converted_address, addr, 6);
    BLE::Instance().securityManager().generateOOB((ble::address_t*)addr);
}

static void cmd_pump(Commander *cmd) {
    cmd->pump();
}

/**
 * Sets up the command shell
 */
void init_commander(void)
{
    _cmd.add("info",       "Get device information.",                                              _cmd_cb_info);
    _cmd.add("advertise",  "Start or stop advertising data. Usage: advertise <on/off>",            _cmd_cb_ble_advertise);
    _cmd.add("scan",       "Scan for devices.               Usage: scan <on/off>",                 _cmd_cb_ble_scan);
    _cmd.add("connect",    "Connect to a device.            Usage: connect <mac>",                 _cmd_cb_ble_connect);
    _cmd.add("remote-oob", "Set OOB keys.                   Usage: oob <hash> <randomizer> <mac>", _cmd_cb_oob);
    _cmd.add("le-oob",     "Generate local LE OOB keys",                                           _cmd_cb_le_oob);

    // prime the serial
    _cmd.init();
}

int main()
{

    /* init the console manager so we can _cmd.printf */
    init_commander();

    _event_queue.call_every(33, cmd_pump, &_cmd);

    conn_dev.run();

    return 0;
}