#ifndef _DVL_SEAPILOT_DRIVER_HPP_
#define _DVL_SEAPILOT_DRIVER_HPP_

#include <iostream>
#include <iodrivers_base/Driver.hpp>
#include <base/Float.hpp>

namespace dvl_seapilot
{
    enum Mode{
        CONFIGURATION = 0,
        MEASUREMENT
    };

    struct Measurement{
        double start_time;
        int sample_number;
        double temperature ;
        double bottom_x;
        double bottom_y;
        double bottom_z;
        double bottom_e;
        double bottom_n;
        double bottom_u;
        double bottom_depth;
        double bottom_heading;
        double bottom_pitch;
        double bottom_roll;
        double mass_x;
        double mass_y;
        double mass_z;
        double mass_e;
        double mass_n;
        double mass_u;
        double mass_depth;
        double mass_heading; 
        double mass_pitch;
        double mass_roll;

        Measurement(){
            start_time = base::unset<double>();
            sample_number = base::unset<int>();
            temperature = base::unset<double>();
            bottom_x = base::unset<double>();
            bottom_y = base::unset<double>();
            bottom_z = base::unset<double>();
            bottom_e = base::unset<double>();
            bottom_n = base::unset<double>();
            bottom_u = base::unset<double>();
            bottom_depth = base::unset<double>();
            bottom_heading = base::unset<double>();
            bottom_pitch = base::unset<double>();
            bottom_roll = base::unset<double>();
            mass_x = base::unset<double>();
            mass_y = base::unset<double>();
            mass_z = base::unset<double>();
            mass_e = base::unset<double>();
            mass_n = base::unset<double>();
            mass_u = base::unset<double>();
            mass_depth = base::unset<double>();
        }


    };


    class Driver : public iodrivers_base::Driver
    {
        std::vector<uint8_t> buffer;
        int extractPacket(uint8_t const *buffer, size_t buffer_size) const;
        public: 
        Driver();
        void open(std::string const& uri);
        bool readMeasurement(Measurement &measurement);
        private:
        Mode current_mode;

        int parsePacket(uint8_t const *buffer, size_t packet_size, Measurement &measurement);
        std::vector<float> getVector(uint8_t *buffer, size_t start, size_t end) const;
        std::vector<std::string> split(uint8_t const* data, size_t const size) const; 
        int getInt(std::string const s) const;
        double getDouble(std::string const s) const;
    };

} // end namespace dvl_seapilot

#endif // _DVL_RTI_DRIVER_HPP__
