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

    protected:
        int extractPacket(uint8_t const *buffer, size_t buffer_size) const;
        int extractConfigurationPacket(uint8_t const *buffer, size_t buffer_size) const;
        
        
        std::vector<uint8_t> buffer;
        bool conf_mode;
        base::Time ensemble_interval;
        VELOCITY_COORDINATE_SYSTEM output_coordinate_system;
    };

} // end namespace dvl_seapilot

#endif // _DVL_RTI_DRIVER_HPP__
