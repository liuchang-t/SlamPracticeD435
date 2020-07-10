#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h" 
#include <iostream>
#include "windows.h"
#include<fstream>
using namespace std;

namespace myslam
{
    class Config
    {
    public:
        std::shared_ptr<Config> config_ = nullptr;
        //cv::FileStorage file_;
        YAML::Node yamlConfig_;

        Config() {} // 

        ~Config();  // close the file when deconstructing 

        // set a new config file 
        void setParameterFile(const std::string& filename);

        // access the parameter values
        template< typename T >
        T get(const std::string& key)
        {
            return yamlConfig_[key].as<T>();
        }
        template< typename T >
        void set(const std::string& key, T m_value)
        {
            Config::yamlConfig_[key] = m_value;
        }
        void saveas(const std::string& filename)
        {
            std::ofstream fout(filename);  // 保存相关改动
            fout << yamlConfig_;
        }
    };
}

#endif // CONFIG_H