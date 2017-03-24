#include "Driver.hpp"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <sys/time.h>
#include <time.h>

namespace dvl_seapilot
{

Driver::Driver() : iodrivers_base::Driver(1000000), output_coordinate_system(INSTRUMENT)
{
    buffer.resize(1000000);
    ensemble_interval = base::Time::fromSeconds(1.0);
}

void Driver::open(std::string const& uri)
{
    openURI(uri);

    conf_mode = true;
    // clean communication channel
    cleanComChannel();

    // send stop command to enter configuration mode
    stopAcquisition();
}

void Driver::read()
{
    int packet_size = readPacket(&buffer[0], buffer.size());
    if (packet_size)
        PD0Parser::parseEnsemble(&buffer[0], packet_size);
}

void Driver::configure()
{
    configureAcquisitionParameters();
    configureProfilingParameters();
}

void Driver::configureAcquisitionParameters()
{
    if (!conf_mode)
        throw std::logic_error("Not in configuration mode. Acquisition has already started.");

    // write ensemble interval
    std::string interval = "CEI 00:";
    interval += ensemble_interval.toString(base::Time::Seconds, "%M:%S");
    interval += '.';
    struct timeval tv = ensemble_interval.toTimeval();
    int u_secs = tv.tv_usec;
    std::string m_secs = boost::lexical_cast<std::string>((int) (u_secs/1000.0));
    if(m_secs.size() > 0)
        interval += m_secs.data()[0];
    else
        interval += '0';
    if(m_secs.size() > 1)
        interval += m_secs.data()[1];
    else
        interval += '0';
    std::cerr << "time: " << interval << std::endl;
    interval += '\r';
    writePacket(reinterpret_cast<uint8_t const*>(interval.data()), interval.size(), 100);
    readConfigurationAck();

    // write output mode
    std::string output_type = "CEOUTPUT 100,";
    output_type += boost::lexical_cast<char>((int)output_coordinate_system);
    output_type += '\r';
    writePacket(reinterpret_cast<uint8_t const*>(output_type.data()), output_type.size(), 100);
    readConfigurationAck();
}

void Driver::configureProfilingParameters()
{
    if (!conf_mode)
        throw std::logic_error("Not in configuration mode. Acquisition has already started.");
        
    // write enable or disable water profiling mode
    std::string water_profile_coomand = "CWPON[0] ";
    water_profile_coomand += boost::lexical_cast<char>((int)mode);
    water_profile_coomand += " 1, 0.000, 0.000\r";
    writePacket(reinterpret_cast<uint8_t const*>(water_profile_coomand.data()), water_profile_coomand.size(), 100);
    readConfigurationAck();

    // write the bin number
    std::string bin_number_command = "CWPBN[0] ";
    bin_number_command += boost::lexical_cast<std::string>((int)bin_number);
    bin_number_command += '\r';
    writePacket(reinterpret_cast<uint8_t const*>(bin_number_command.data()), bin_number_command.size(), 100);
    readConfigurationAck();

    // write the bin size
    std::string bin_size_command = "CWPBS[0] ";
    bin_size_command += boost::lexical_cast<std::string>((double)bin_size);
    bin_size_command += '\r';
    writePacket(reinterpret_cast<uint8_t const*>(bin_size_command.data()), bin_size_command.size(), 100);
    readConfigurationAck();

    // write the blank size
    std::string blank_lenght_command = "CWPBL[0] ";
    blank_lenght_command += boost::lexical_cast<std::string>((double)blank_lenght);
    blank_lenght_command += '\r';
    writePacket(reinterpret_cast<uint8_t const*>(blank_lenght_command.data()), blank_lenght_command.size(), 100);
    readConfigurationAck();
}

void Driver::startAcquisition()
{
    // write start
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

void Driver::cleanComChannel()
{
    try
    {
        writePacket(reinterpret_cast<uint8_t const*>("\r"), 1, 100);
        readConfigurationAck();
    }
    catch (std::runtime_error e) {}
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

void Driver::setEnsembleInterval(double seconds)
{
    this->ensemble_interval = base::Time::fromSeconds(seconds);
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

void Driver::setBinSize(double bin_size)
{
    this->bin_size = bin_size;
}

void Driver::setBinNumber(uint8_t bin_number)
{
    this->bin_number = bin_number;
}

void Driver::setBlankLenght(double blank_lenght)
{
    this->blank_lenght = blank_lenght;
}

void Driver::setOperatingMode(OPERATING_MODE mode)
{
    this->mode = mode;
}

}
