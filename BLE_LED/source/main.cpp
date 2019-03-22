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
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "ble/gap/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"
#include "pretty_printer.h"
#include "LEDService.h"

const static char DEVICE_NAME[] = "LED";

static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

static DiscoveredCharacteristic led_characteristic;
static bool trigger_led_characteristic = false;

void service_discovery(const DiscoveredService *service) {
    if (service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
        printf("S UUID-%x attrs[%u %u]\r\n", service->getUUID().getShortUUID(), service->getStartHandle(), service->getEndHandle());
    } else {
        printf("S UUID-");
        const uint8_t *longUUIDBytes = service->getUUID().getBaseUUID();
        for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
            printf("%02x", longUUIDBytes[i]);
        }
        printf(" attrs[%u %u]\r\n", service->getStartHandle(), service->getEndHandle());
    }
}

void update_led_characteristic(void) {
    if (!BLE::Instance().gattClient().isServiceDiscoveryActive()) {
        led_characteristic.read();
    }
}

void characteristic_discovery(const DiscoveredCharacteristic *characteristicP) {
    printf("  C UUID-%x valueAttr[%u] props[%x]\r\n", characteristicP->getUUID().getShortUUID(), characteristicP->getValueHandle(), (uint8_t)characteristicP->getProperties().broadcast());
    if (characteristicP->getUUID().getShortUUID() == 0xa001) { /* !ALERT! Alter this filter to suit your device. */
        led_characteristic        = *characteristicP;
        trigger_led_characteristic = true;
    }
}

void discovery_termination(Gap::Handle_t connectionHandle) {
    printf("terminated SD for handle %u\r\n", connectionHandle);
    if (trigger_led_characteristic) {
        trigger_led_characteristic = false;
        event_queue.call(update_led_characteristic);
    }
}

void trigger_toggled_write(const GattReadCallbackParams *response) {
    if (response->handle == led_characteristic.getValueHandle()) {
        printf("trigger_toggled_write: handle %u, offset %u, len %u\r\n", response->handle, response->offset, response->len);
        for (unsigned index = 0; index < response->len; index++) {
            printf("%c[%02x]", response->data[index], response->data[index]);
        }
        printf("\r\n");

        uint8_t toggledValue = response->data[0] ^ 0x1;
        led_characteristic.write(1, &toggledValue);
    }
}

void trigger_read(const GattWriteCallbackParams *response) {
    if (response->handle == led_characteristic.getValueHandle()) {
        led_characteristic.read();
    }
}

class LEDDemo : ble::Gap::EventHandler {
public:
    LEDDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _alive_led(LED1, 1),
        _actuated_led(LED2, 0),
        _is_connecting(false),
        _led_uuid(LEDService::LED_SERVICE_UUID),
        _led_service(NULL),
        _adv_data_builder(_adv_buffer) { }

    virtual ~LEDDemo() {
        delete _led_service;
    }

    virtual void start() {
        ble_error_t error = _ble.gap().startScan();
        if (error) {
            print_error(error, "Error caused by Gap::startScan");
            return;
        }
        start_advertising();
    }

    void stop() {
        ble_error_t error = _ble.gap().stopScan();
        if (error) {
            print_error(error, "Error caused by Gap::stopScan");
            return;
        }
        _is_connecting = false;
        stop_advertising();
    }

    void init() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &LEDDemo::on_init_complete);

        _event_queue.call_every(500, this, &LEDDemo::blink);

        _event_queue.dispatch_forever();
    }

    void stop_advertising() {

    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _adv_data_builder.setFlags();
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&_led_uuid, 1));
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            printf("_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            printf("_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }

        /* Start advertising */
        printf("Starting advertisement\r\n");
        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            printf("_ble.gap().startAdvertising() failed\r\n");
            return;
        }
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent& event) {
        _ble.gattClient().onServiceDiscoveryTermination(discovery_termination);
        _ble.gattClient().launchServiceDiscovery(
            event.getConnectionHandle(),
            service_discovery,
            characteristic_discovery,
            BLE_UUID_UNKNOWN,
            BLE_UUID_UNKNOWN
        );
        _is_connecting = false;
    }

    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        printf("Init complete\r\n");
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

        _led_service = new LEDService(_ble, false);

        _ble.gattServer().onDataWritten(this, &LEDDemo::on_data_written);
        _ble.gattClient().onDataRead(trigger_toggled_write);

        printf("Setting scan parameters\r\n");
        ble::ScanParameters scan_params;
        _ble.gap().setScanParameters(scan_params);
        printf("Finished init code\r\n");

        start();
    }

    void blink() {
        _alive_led = !_alive_led;
    }

    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _is_connecting = false;
    }

    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {
        /* don't bother with analysing scan result if we're already connecting */
        if (_is_connecting) {
            return;
        }

        ble::AdvertisingDataParser adv_data(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_data.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_data.next();

            /* connect to a discoverable device */
            if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME &&
                field.value.size() == strlen(DEVICE_NAME) &&
                (memcmp(field.value.data(), DEVICE_NAME, field.value.size()) == 0)) {

                printf("Adv from: ");
                printf("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                       event.getPeerAddress().data()[5],
                       event.getPeerAddress().data()[4],
                       event.getPeerAddress().data()[3],
                       event.getPeerAddress().data()[2],
                       event.getPeerAddress().data()[1],
                       event.getPeerAddress().data()[0]);
                printf(" rssi: %d, scan response: %u, connectable: %u\r\n",
                       event.getRssi(), event.getType().scan_response(), event.getType().connectable());

                ble_error_t error = _ble.gap().stopScan();

                if (error) {
                    print_error(error, "Error caused by Gap::stopScan");
                    return;
                }

                const ble::ConnectionParameters connection_params;

                error = _ble.gap().connect(
                    event.getPeerAddressType(),
                    event.getPeerAddress(),
                    connection_params
                );

                if (error) {
                    _ble.gap().startScan();
                    return;
                }

                /* we may have already scan events waiting
                 * to be processed so we need to remember
                 * that we are already connecting and ignore them */
                _is_connecting = true;

                return;
            }
        }
    }

    /**
     * This callback allows the LEDService to receive updates to the ledState Characteristic.
     *
     * @param[in] params Information about the characterisitc being updated.
     */
    void on_data_written(const GattWriteCallbackParams *params) {
        // trigger_read(params);
        if ((params->handle == _led_service->getValueHandle()) && (params->len == 1)) {
            _actuated_led = *(params->data);
        }
    }

protected:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _alive_led;
    DigitalOut _actuated_led;
    bool _is_connecting;

    UUID _led_uuid;
    LEDService *_led_service;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    LEDDemo demo(ble, event_queue);
    demo.init();

    return 0;
}

