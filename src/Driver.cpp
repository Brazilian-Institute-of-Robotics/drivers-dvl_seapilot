#include "Driver.hpp"
#include <iostream>
#include <boost/lexical_cast.hpp>

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
    output_type += boost::lexical_cast<char>((int)output_coordinate_system);
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
    
    if(packet_size == sizeof(ConfigurationAck))
    {
        ConfigurationAck const& ack = *reinterpret_cast<ConfigurationAck const*>(&buffer[0]);
        if(ack.echo == ConfigurationAck::ACK) // received ACK
            return;
        else if(ack.echo == ConfigurationAck::NAK) // received NAK
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
    if(conf_mode)
        return extractConfigurationPacket(buffer, buffer_size);
    else
        return PD0Parser::extractPacket(buffer, buffer_size);
}

int Driver::extractConfigurationPacket(const uint8_t* buffer, size_t buffer_size) const
{
    if(buffer_size < sizeof(ConfigurationAck))
    {
        // to less data, wait for new data
        return 0;
    }
    
    // search for the carriage return <CR> and linefeed <LF>
    size_t packet_end;
    for(packet_end = 1; packet_end < buffer_size; packet_end++)
    {
        if(buffer[packet_end-1] == ConfigurationAck::CR &&
            buffer[packet_end] == ConfigurationAck::LF)
            break;
    }

    if(packet_end == buffer_size)
    {
        // no packet end in buffer, wait for new data
        return 0;
    }
    else if(packet_end > sizeof(ConfigurationAck)-1)
    {
        // realign the IODriver buffer to the start of the candidate packet
        return -(packet_end-(sizeof(ConfigurationAck)-1));
    }
    else if(buffer[0] != ConfigurationAck::ACK && buffer[0] != ConfigurationAck::NAK)
    {
        // not actually a packet. Drop the msg and let IODriver call
        // us back
        return -sizeof(ConfigurationAck);
    }
    
    return sizeof(ConfigurationAck);
}

}