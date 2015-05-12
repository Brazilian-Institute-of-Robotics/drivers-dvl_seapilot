#include <iostream>
#include <dvl_seapilot/Driver.hpp>
#include <dvl_teledyne/PD0Messages.hpp>

void usage()
{
    std::cerr << "dvl_seapilot_read DEVICE" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        usage();
        return 1;
    }

    dvl_seapilot::Driver driver;
    driver.open(argv[1]);
    driver.setReadTimeout(base::Time::fromSeconds(5));
    driver.startAcquisition();

    while(true)
    {
        driver.read();

        dvl_teledyne::BottomTracking const& tracking = driver.bottomTracking;
        std::cout << tracking.time.toString() << " " << driver.status.seq;
        for (int beam = 0; beam < 4; ++beam)
            std::cout << " " << tracking.range[beam] << " " << tracking.velocity[beam] << " " << tracking.evaluation[beam];
        std::cout << std::endl;
    }


    return 0;
}
