#include "Driver.hpp"
#include <iostream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <fstream>

namespace dvl_seapilot
{
    Driver::Driver()
        : iodrivers_base::Driver(1000000){
            buffer.resize(1000000);
    }

    void Driver::open(std::string const& uri)
    {
        openURI(uri);
    }


    int Driver::extractPacket(uint8_t const *buffer, size_t buffer_size) const{
        if(current_mode == CONFIGURATION){
            return -buffer_size;
        } else if(current_mode == MEASUREMENT){
            //std::cout << "im measurement mode"  << std::endl;

            for(int i = 0; i < buffer_size; i++){
                //std::cout << "I: " << i  << std::endl;
                //std::cout << "BUFFER[I]: " << buffer[i] << std::endl; 
               
                //std::cout << "checksum: " << buffer[54] << buffer[55] << std::endl;  
                //The char at position i is the start char of the packet.
                if (buffer[i] == '$'){
                    //std::cout << "found start_signal" << std::endl;
                    //the startchar is not on position 0. return -i.
                    //the buffer will move the startchar to position 0
                    if(i > 0){
                        //std::cout << "return(-i) " << -i << std::endl;
                        return -i;
                    }

                    int check_sum = 0;
                    for(int j = 1; j < 85; j++){
                        //found not the end of package.
                        //return 0 to search again after ther are more
                        //data 
                        //std::cout << "J: " << j  << std::endl;
                        //std::cout << "BUFFER[J]: " << buffer[j] << std::endl; 
                        if(j > buffer_size){
                            //std::cout << "return(0) " << 0 << std::endl;
                            return 0;
                        }
                        
                        //found a new startchar but no end of the packet.
                        //use this cha as new startchar
                        if(buffer[j] == '$'){
                            //std::cout << "return(-j) " << -j << std::endl;
                            return -j;
                        }

                        //the following test needs j >= 5
                        if(j>=6){
                            //std::cout << "BUFFER[J-5]: " << buffer[j-5] << std::endl; 
                            //calculate checksume
                            check_sum = check_sum ^ buffer[j-5];
                            //std::cout << "CALCULATED CHECK_SUM: " << check_sum << std::endl;
                            
                            int packet_check_sum;
                            char input[2];
                            input[0] = buffer[j-3];
                            input[1] = buffer[j-2];
                            
                            sscanf(input, "%x", &packet_check_sum);
                            //std::cout << "READED CHECK_SUM: " << packet_check_sum << std::endl;

                            //found the end of the package.
                            //return the size of the package.
                            if(packet_check_sum == check_sum && buffer[j-4] == '*'&& buffer[j-1] == '\r' && buffer[j] =='\n'){
                            //if(buffer[j-4] == '*' && buffer[j-1] == '\r' && buffer[j] == '\n'){
                                //std::cout << "return(j) " << j << std::endl;
                                return j;
                            }
                        }

                    }


                }
            }
        }

        //std::cout << "return(-buffer_size) " << -(int)buffer_size << std::endl;
        return -(int)buffer_size;
    }

    bool Driver::readMeasurement(Measurement &measurement){
        current_mode = MEASUREMENT;
        Measurement internal_measurement;
        base::Time timeout = base::Time().fromSeconds(10);
        bool measurement_complete = false;
        while(!measurement_complete){
            int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
            int packet_type;
            if(packet_size > 0){ 
                packet_type = this->parsePacket(&buffer[0], packet_size, internal_measurement);
            }
            if(packet_type == 30){
                measurement_complete = true;
            }
        }



        measurement = internal_measurement;
        return true;
    }

    bool Driver::startMeasurement(){
	current_mode = CONFIGURATION;
        last_command = "START\r";
	writePacket(reinterpret_cast<uint8_t const*>("START\r"), 6, 100);
        base::Time timeout = base::Time().fromSeconds(10);
        int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
        return true;
    }

    bool Driver::stopMeasurement(){
        writePacket(reinterpret_cast<uint8_t const*>("STOP\r"), 5, 100);
        return true;
    }


    int Driver::parsePacket(uint8_t const *buffer, size_t packet_size, Measurement &measurement){
        std::vector<std::string> packet = this->split(buffer, packet_size);

        std::string packet_type = packet.at(0).substr(5,2);



        switch(this->getInt(packet_type)){
            case 1:
                //start_time
                measurement.sample_number  = this->getInt(packet.at(2));
                measurement.temperature    = this->getDouble(packet.at(3))/100;
                measurement.bottom_x       = this->getDouble(packet.at(4))/1000;
                measurement.bottom_y       = this->getDouble(packet.at(5))/1000;
                measurement.bottom_z       = this->getDouble(packet.at(6))/1000;
                measurement.bottom_depth   = this->getDouble(packet.at(7))/1000;
                measurement.mass_x         = this->getDouble(packet.at(8))/1000;
                measurement.mass_y         = this->getDouble(packet.at(9))/1000;
                measurement.mass_z         = this->getDouble(packet.at(10))/1000;
                measurement.mass_depth     = this->getDouble(packet.at(11))/1000;
                return 1;
            case 2:
                //start_time
                measurement.bottom_e       = this->getDouble(packet.at(4))/1000;
                measurement.bottom_n       = this->getDouble(packet.at(5))/1000;
                measurement.bottom_u       = this->getDouble(packet.at(6))/1000;
                measurement.mass_e         = this->getDouble(packet.at(8))/1000;
                measurement.mass_n         = this->getDouble(packet.at(9))/1000;
                measurement.mass_u         = this->getDouble(packet.at(10))/1000;
                return 2;
            case 30:
                measurement.bottom_heading = this->getDouble(packet.at(1).substr(0,1)) + (this->getDouble(packet.at(1).substr(2, 3)))/1000; 
                measurement.bottom_pitch   = this->getDouble(packet.at(2).substr(0,1)) + (this->getDouble(packet.at(2).substr(2, 3)))/1000; 
                measurement.bottom_roll    = this->getDouble(packet.at(3).substr(0,1)) + (this->getDouble(packet.at(3).substr(2, 3)))/1000; 

                return 30;
            case 31:
                measurement.mass_heading   = this->getDouble(packet.at(1).substr(0,1)) + (this->getDouble(packet.at(1).substr(2, 3)))/1000; 
                measurement.mass_pitch     = this->getDouble(packet.at(2).substr(0,1)) + (this->getDouble(packet.at(2).substr(2, 3)))/1000; 
                return 31;
        }
    }

    std::vector<std::string> Driver::split(uint8_t const *data, size_t const size) const{
        char const* buffer_as_string = reinterpret_cast<char const*>(data);
        std::string s = std::string(buffer_as_string, size);
        std::vector<std::string> splitted;
        boost::split(splitted, s,  boost::algorithm::is_any_of(",")); 
        return splitted; 
    }


    int Driver::getInt(std::string const s) const {
        int value;
        std::string s_tmp = s;
        boost::algorithm::trim_if(s_tmp, boost::is_any_of("[*]"));
        std::stringstream ss(s_tmp);
        if (!(ss >> value)){
            std::stringstream error_string;
            error_string << "Expected an r response, but read " << s << std::flush;
        }

        return value;
    }

    double Driver::getDouble(std::string const s) const {
        int int_value;
        double value;
        if(s.size() > 0){
            std::string s_tmp = s;
            boost::algorithm::trim_if(s_tmp, boost::is_any_of("[*]"));
            std::stringstream ss(s_tmp);
            if (!(ss >> int_value)){
                std::stringstream error_string;
                error_string << "Expected an r response, but read " << s << std::flush;
            }
            value = (double)int_value;
            if (int_value == -99999){
                value = base::unset<double>();
            }
        } else{
            value = base::unset<double>();
        }


        return value;
    }
}
#include "Driver.hpp"
