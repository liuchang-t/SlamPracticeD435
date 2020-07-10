#include "../include/config.h"

namespace myslam
{

    void Config::setParameterFile(const std::string& filename)
    {
        if (config_ == nullptr)
            config_ = shared_ptr<Config>(new Config);
        yamlConfig_ = YAML::LoadFile(filename);
        if (yamlConfig_.IsNull())
        {
            std::cerr << "parameter file " << filename << " does not exist." << std::endl;
            yamlConfig_.reset();
            return;
        }
    }

    Config::~Config()
    {
        /*if ( file_.isOpened() )
            file_.release();*/
    }

}