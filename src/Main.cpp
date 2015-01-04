#include <iostream>
#include <dvl_seapilot/Driver.hpp>

int main(int argc, char** argv)
{
    std::cout << "instanciate driver" << std::endl;
    dvl_seapilot::Driver driver;

    std::cout << "open devive" << std::endl;
    driver.open(argv[1]);

    dvl_seapilot::Measurement measurement;
        
    std::cout << "open devive" << std::endl;
    while(driver.readMeasurement(measurement)){
        std::cout << "READET_PACKET:" << std::endl;
        std::cout << "- START_TIME:     " << measurement.start_time << std::endl;
        std::cout << "- SAMPLE-NUMBER:  " << measurement.sample_number << std::endl;
        std::cout << "- TEMPERATURE:    " << measurement.temperature << std::endl;
        std::cout << "- BOTTOM_X:       " << measurement.bottom_x << std::endl;
        std::cout << "- BOTTOM_Y:       " << measurement.bottom_y << std::endl;
        std::cout << "- BOTTOM_Z:       " << measurement.bottom_z << std::endl;
        std::cout << "- BOTTOM_E:       " << measurement.bottom_e << std::endl;
        std::cout << "- BOTTOM_N:       " << measurement.bottom_n << std::endl;
        std::cout << "- BOTTOM_U:       " << measurement.bottom_u << std::endl;
        std::cout << "- BOTTOM_DEPTH:   " << measurement.bottom_depth << std::endl;
        std::cout << "- BOTTOM_HEADING: " << measurement.bottom_heading << std::endl;
        std::cout << "- BOTTOM_PITCH:   " << measurement.bottom_pitch << std::endl;
        std::cout << "- BOTTOM_ROLL:    " << measurement.bottom_roll << std::endl;
        std::cout << "- MASS_X:         " << measurement.mass_x << std::endl;
        std::cout << "- MASS_Y:         " << measurement.mass_y << std::endl;
        std::cout << "- MASS_Z:         " << measurement.mass_z << std::endl;
        std::cout << "- MASS_E:         " << measurement.mass_e << std::endl;
        std::cout << "- MASS_N:         " << measurement.mass_n << std::endl;
        std::cout << "- MASS_U:         " << measurement.mass_u << std::endl;
        std::cout << "- MASS_DEPTH:     " << measurement.mass_depth << std::endl;

    }


    return 0;
}
