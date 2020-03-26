#ifndef MIE443_CONTEST2_LOGGER_H
#define MIE443_CONTEST2_LOGGER_H

#include <fstream>

class Logger
{
 private:
    std::ofstream logfile;
 
 public:
    void open(const std::string& filepath) 
    { 
       logfile.open(filepath, std::ofstream::out | std::ofstream::app); 
       logfile << "\n************ NEW RUN *************\n"; 
    }

    ~Logger() 
    { 
       logfile << "************ END *************\n"; 
       logfile.close(); 
    }
    
    void write(const std::string& data) 
    { 
       logfile << data << std::endl; 
    }
};

#endif //MIE443_CONTEST2_LOGGER_H
