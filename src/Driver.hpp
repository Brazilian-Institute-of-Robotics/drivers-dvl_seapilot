#ifndef _DVL_SEAPILOT_DRIVER_HPP_
#define _DVL_SEAPILOT_DRIVER_HPP_

#include <iostream>
#include <iodrivers_base/Driver.hpp>
#include <base/Float.hpp>
#include <dvl_teledyne/PD0Parser.hpp>
#include <stdint.h>
#include <boost/static_assert.hpp>

namespace dvl_seapilot
{


    class Driver : public iodrivers_base::Driver, public dvl_teledyne::PD0Parser
    {
    public:
        enum VELOCITY_COORDINATE_SYSTEM
        {
            BEAM = 0, INSTRUMENT, EARTH, SHIP
        };

        enum OPERATING_MODE
        {
            DVL = 0, PROFILE
        };

        struct ConfigurationAck
        {
            enum CONFIGURATION_PROTOCOL { ACK = 0x06, NAK = 0x15, CR = 0x0d, LF = 0x0a};
            uint8_t echo;
            uint16_t msg_end; // carriage return + linefeed
        } __attribute__((packed));
        BOOST_STATIC_ASSERT(sizeof(ConfigurationAck) == 3);

        Driver();

        /** Tries to access the DVL at the provided URI
         *
         * For now, only a serial port can be provided. It is assumed that the
         * DVL is using 115200 bauds (the manufacturer's default)
         */
        void open(std::string const& uri);

        /** Configure the device according the parameters specified */
        void configure();

        /** Configure acquisition related paramenters */
        void configureAcquisitionParameters();

        /** Configure water profiling related parameters */
        void configureProfilingParameters();

        /** Start acquisition
         *
         * Since the driver relies on receiving PD0 message frames, this method
         * requires the DVL to send into this format, and then starts pinging
         */
        void startAcquisition();

        /** Stops acquisition and switches to configuration mode
         */
        void stopAcquisition();

        /** Read available packets on the I/O */
        void read();

        /** Verifies that the DVL acked a configuration command
         *
         * Throws std::runtime_error if an error is reported by the device
         */
        void readConfigurationAck(base::Time const& timeout = base::Time::fromSeconds(1));

        /** Configures the output coordinate system */
        void setOutputConfiguration(VELOCITY_COORDINATE_SYSTEM output_coordinate_system);

        void setEnsembleInterval(double seconds);
        void setBinSize(double bin_size);
        void setBinNumber(uint8_t bin_number);
        void setBlankLenght(double blank_lenght);
        void setOperatingMode(OPERATING_MODE mode);

    protected:
        int extractPacket(uint8_t const *buffer, size_t buffer_size) const;
        int extractConfigurationPacket(uint8_t const *buffer, size_t buffer_size) const;
        void cleanComChannel();


        std::vector<uint8_t> buffer;
        bool conf_mode;
        base::Time ensemble_interval;
        VELOCITY_COORDINATE_SYSTEM output_coordinate_system;

        double bin_size;
        uint8_t  bin_number;
        double blank_lenght;

        // Enables or disable water profiling
        OPERATING_MODE mode;

    };

} // end namespace dvl_seapilot

#endif // _DVL_RTI_DRIVER_HPP__
