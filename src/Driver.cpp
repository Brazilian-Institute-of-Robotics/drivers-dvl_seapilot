#include "Driver.hpp"
#include <iostream>

namespace dvl_seapilot
{

Driver::Driver() : iodrivers_base::Driver(1000000), output_coordinate_system(INSTRUMENT)
{
    buffer.resize(1000000);
}

void Driver::open(std::string const& uri)
{
    openURI(uri);
    
    stopAcquisition();
}

void Driver::read()
{
    int packet_size = readPacket(&buffer[0], buffer.size());
    if (packet_size)
        PD0Parser::parseEnsemble(&buffer[0], packet_size);
}
    
void Driver::startAcquisition()
{
    if (!conf_mode)
        throw std::logic_error("Not in configuration mode. Acquisition has already started.");

    std::string output_type = "CEOUTPUT 100,";
    output_type += (uint8_t)(output_coordinate_system);
    output_type += '\r';
    writePacket(reinterpret_cast<uint8_t const*>(output_type.data()), output_type.size(), 100);
    readConfigurationAck();
    writePacket(reinterpret_cast<uint8_t const*>("START\r"), 6, 100);
    readConfigurationAck();
    conf_mode = false;
}

void Driver::stopAcquisition()
{
    writePacket(reinterpret_cast<uint8_t const*>("STOP\r"), 5, 100);
    conf_mode = true;
    readConfigurationAck();
}

void Driver::readConfigurationAck(const base::Time& timeout)
{
    if (!conf_mode)
        throw std::logic_error("Not in configuration mode. Acquisition has already started.");
    int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
    if (packet_size == 3 && buffer[1] == 0x0d && buffer[2] == 0x0a)
    {
        if(buffer[0] == 0x06) // received ACK
            return;
        else if(buffer[1] == 0x15) // received NAK
            throw std::runtime_error("The device received an invalid command.");
    }
    throw std::runtime_error("Received unknown message from device.");
}

void Driver::setOutputConfiguration(VELOCITY_COORDINATE_SYSTEM output_coordinate_system)
{
    this->output_coordinate_system = output_coordinate_system;
}

int Driver::extractPacket(const uint8_t* buffer, size_t buffer_size) const
{
    return PD0Parser::extractPacket(buffer, buffer_size);
}

}