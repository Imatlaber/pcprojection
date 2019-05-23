/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: wrapper of bot param
 */

#ifndef _GTK_PARAM_HELPER_HPP_
#define _GTK_PARAM_HELPER_HPP_

#include <string>
#include <map>
#include <gtk/gtk.h>
#include <iostream>

#include <gtkmm.h>
#include <glibmm/keyfile.h>

class GtkParametersHelper
{
private:
    std::map<std::string, std::string> saved_params_;

public:
    void loadParameters(GKeyFile *keyfile, std::string group_name)
    {
        Glib::KeyFile gkeyfile(keyfile);
        if(gkeyfile.has_group(group_name)) {
            std::vector<std::string> keys = gkeyfile.get_keys(group_name);
            for(size_t i=0; i<keys.size(); i++) {
                std::string value = gkeyfile.get_value(group_name, keys[i]);
                saved_params_[keys[i]] = value;
            }
        }
    }

    std::string getStringParam(const std::string& key, const std::string& default_val)
    {
        std::string val;
        if(saved_params_.find(key) != saved_params_.end())
            val = saved_params_[key];
        else
            val = default_val;

        return val;
    }

    double getDoubleParam(const std::string& key, const double default_val)
    {
        double val;
        if(saved_params_.find(key) != saved_params_.end()) {
            try {
                std::string::size_type sz;
                val = stod(saved_params_[key], &sz);
            }
            catch (const std::out_of_range &oor) {
                std::cout<<"Parameter "<<key<<" out of double range. Assinging to default value of "<<default_val<<std::endl;
                val = default_val;
            }
        }
        else
            val = default_val;

        return val;
    }

    bool getBoolParam(const std::string& key, const bool default_val)
    {
        bool val;
        if(saved_params_.find(key)!= saved_params_.end()) {
            if(saved_params_[key] == "true")
                val = true;
            else
                // TODO better check for == "false" and if fail -> return default
                val = false;
        }
        else
            val = default_val;

        return val;
    }

    void setParamsToKeyfile(const std::string& group_name, Glib::KeyFile& keyfile)
    {
        for(const auto& key : saved_params_)
            keyfile.set_string(group_name, key.first, key.second);
    }
};

#endif
