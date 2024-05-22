#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/BatteryService.h"
#include "ble/services/DeviceInformationService.h"
#include "pretty_printer.h"
#include "XNucleoIKS01A3.h"

class EnvironmentalService {
public:
    typedef int16_t  TemperatureType_t;
    typedef uint16_t HumidityType_t;
    typedef uint32_t PressureType_t;

    /**
     * @brief   EnvironmentalService constructor.
     * @param   _ble Reference to BLE device.
     */
    EnvironmentalService(BLE& _ble) :
        ble(_ble),
        temperatureCharacteristic(GattCharacteristic::UUID_TEMPERATURE_CHAR, &temperature,GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
        humidityCharacteristic(GattCharacteristic::UUID_HUMIDITY_CHAR, &humidity,GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
        pressureCharacteristic(GattCharacteristic::UUID_PRESSURE_CHAR, &pressure,GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
    {
        static bool serviceAdded = false; /* We should only ever need to add the information service once. */
        if (serviceAdded) {
            return;
        }

        GattCharacteristic *charTable[] = { &humidityCharacteristic,
                                            &pressureCharacteristic,
                                            &temperatureCharacteristic };

        GattService environmentalService(GattService::UUID_ENVIRONMENTAL_SERVICE, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        ble.gattServer().addService(environmentalService);
        serviceAdded = true;
    }

    /**
     * @brief   Update humidity characteristic.
     * @param   newHumidityVal New humidity measurement.
     */
    void updateHumidity(HumidityType_t newHumidityVal)
    {
        humidity = (HumidityType_t) (newHumidityVal * 100);
        ble.gattServer().write(humidityCharacteristic.getValueHandle(), (uint8_t *) &humidity, sizeof(HumidityType_t));
    }

    /**
     * @brief   Update pressure characteristic.
     * @param   newPressureVal New pressure measurement.
     */
    void updatePressure(PressureType_t newPressureVal)
    {
        pressure = (PressureType_t) (newPressureVal * 10);
        ble.gattServer().write(pressureCharacteristic.getValueHandle(), (uint8_t *) &pressure, sizeof(PressureType_t));
    }

    /**
     * @brief   Update temperature characteristic.
     * @param   newTemperatureVal New temperature measurement.
     */
    void updateTemperature(float newTemperatureVal)
    {
        temperature = (TemperatureType_t) (newTemperatureVal * 100);
        ble.gattServer().write(temperatureCharacteristic.getValueHandle(), (uint8_t *) &temperature, sizeof(TemperatureType_t));
    }

private:
    BLE& ble;

    TemperatureType_t temperature;
    HumidityType_t    humidity;
    PressureType_t    pressure;

    ReadOnlyGattCharacteristic<TemperatureType_t> temperatureCharacteristic;
    ReadOnlyGattCharacteristic<HumidityType_t>    humidityCharacteristic;
    ReadOnlyGattCharacteristic<PressureType_t>    pressureCharacteristic;
};


/* device name */
const static char DEVICE_NAME[] = "EnvironmentalSensor";

/* list of services */
#define UUID_ENVIRONMENTAL_SERVICE  0x181A
#define UUID_DEVICE_INFORMATION_SERVICE 0x180A
UUID uuid_list[] = {
    UUID_ENVIRONMENTAL_SERVICE,
    UUID_DEVICE_INFORMATION_SERVICE
    };

/* Advertising data buffer */
uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];

/* BLE event queue */
static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

bool initFlag = false;

XNucleoIKS01A3 *sensors = XNucleoIKS01A3::instance(D14, D15);

Ticker updateSensors;

// Declare and define a measurement update flag
bool sensorFlag = false;

void updateMeasurments(){
    sensorFlag = true;
}

void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("Ble initialization failed.");
        initFlag = false;
        return;
    }
    initFlag = true;
}

class EnvironmentalApp : ble::Gap::EventHandler {
public:
    EnvironmentalApp(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),       
        _connected(false),
        _envService(ble),
        _deviceService(ble)
        {}
    
    void update_sensor_value() {
        if (_connected) {
            float t_value;
            float p_value;
            float h_value;

            /* Read the environmental sensors */
            sensors->t_sensor->get_temperature( &t_value );
            sensors->pt_sensor->get_pressure( &p_value);
            sensors->ht_sensor->get_humidity( &h_value );

            _envService.updateTemperature( t_value );
            _envService.updateHumidity( h_value );
            _envService.updatePressure( p_value );
        }
    }

private:
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        _connected = false;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;

    bool _connected;

    EnvironmentalService _envService;
    DeviceInformationService _deviceService;
};

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

void start_advertising(BLE &ble) {

    /* Create AdvertisingDataBuilder object */
    ble::AdvertisingDataBuilder builder(adv_buffer);    

    ble::adv_interval_t min( 1000 );
    ble::adv_interval_t max( 2000 );
    ble::AdvertisingParameters advParam(  ble::advertising_type_t::CONNECTABLE_UNDIRECTED, ble::adv_interval_t(ble::millisecond_t(1000)) );

    builder.setAppearance(ble::adv_data_appearance_t::GENERIC_HEART_RATE_SENSOR );
    builder.setName( "Environmental Sensor");
    builder.setLocalServiceList( uuid_list );
    
    /* Set advertising parameters*/
    ble_error_t error = ble.gap().setAdvertisingParameters(
        ble::LEGACY_ADVERTISING_HANDLE,
        advParam
    );

    if (error) {
        printf("_ble.gap().setAdvertisingParameters() failed\r\n");
        return;
    }

    /* set Advertising payload */
    error = ble.gap().setAdvertisingPayload(
        ble::LEGACY_ADVERTISING_HANDLE,
        builder.getAdvertisingData()
    );

    if (error) {
        printf("_ble.gap().setAdvertisingPayload() failed\r\n");
        return;
    }

    /* Start advertising */
    error = ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    if (error) {
        printf("_ble.gap().startAdvertising() failed\r\n");
        return;
    }
}


int main()
{

    BLE& mydevice = BLE::Instance();

    mydevice.onEventsToProcess(schedule_ble_events);

    EnvironmentalApp *eventHandler = new EnvironmentalApp(mydevice, event_queue);
    Gap& myGap = mydevice.gap();
    myGap.setEventHandler((ble::Gap::EventHandler *) eventHandler);

    mydevice.init(&on_init_complete);

    while (1) {
 
        if( sensorFlag ){
            eventHandler->update_sensor_value();
            sensorFlag = false;
        }

        if( initFlag ){
            print_mac_address(); 
            start_advertising(mydevice);

            sensors->t_sensor->enable();
            sensors->ht_sensor->enable();
            sensors->pt_sensor->enable();

            updateSensors.attach( &updateMeasurments, 1);

            initFlag = false;
        }

        /* check for BLE events */
        event_queue.dispatch(100);
    }
    return 0;
}



