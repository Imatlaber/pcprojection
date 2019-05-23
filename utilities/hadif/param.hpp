/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: wrapper of bot param
 */

#ifndef PARAM_H_
#define PARAM_H_

#include <iostream>
#include <string>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include <assert.h>
#include <unistd.h>
#include "file_helper.hpp"

class GenericFormat {

    /**
     * message in case string conversion is not available
     */
public:
    static std::string failureMessage() {
        return std::string("<toString not available>");
    }

    /**
     * fallback option
     */
public:
    template<class type>
    static std::string toString(type /*value*/) {
        return failureMessage();
    }

    /**
     * generic format for boolean
     */
    public:
        static std::string toString(const bool& value) {
            return value ? "true" : "false";
        }

    /**
     * use of std::to_string
     */

#define _GENERICFORMAT_STD_TO_STRING_(TYPE)  \
        public:\
            static std::string toString(const TYPE& value) {\
                return std::to_string(value);\
            }

    // detailed list is necessary to prevent template function
    _GENERICFORMAT_STD_TO_STRING_(char)
    _GENERICFORMAT_STD_TO_STRING_(unsigned char)

    _GENERICFORMAT_STD_TO_STRING_(short)
    _GENERICFORMAT_STD_TO_STRING_(unsigned short)

    _GENERICFORMAT_STD_TO_STRING_(int)
    _GENERICFORMAT_STD_TO_STRING_(unsigned int)

    _GENERICFORMAT_STD_TO_STRING_(long)
    _GENERICFORMAT_STD_TO_STRING_(unsigned long)

    _GENERICFORMAT_STD_TO_STRING_(float)
    _GENERICFORMAT_STD_TO_STRING_(double)
    _GENERICFORMAT_STD_TO_STRING_(long double)


    /**
     * use with std::string constructor
     */

#define _GENERICFORMAT_STD_STRING_(TYPE)  \
        public:\
            static std::string toString(TYPE value) {\
                return std::string(value);\
            }

    _GENERICFORMAT_STD_STRING_(char*)
    _GENERICFORMAT_STD_STRING_(const char*)
    _GENERICFORMAT_STD_STRING_(std::string)


};

class param_t
{
protected:
		
	lcm::LCM * lcm_ ;
	BotParam * param_ ;
	std::string ns_ ;

public:
    param_t(lcm::LCM* lcm, bool keep_updated=false) :
        lcm_(lcm),
		param_(nullptr)
    {
		assert(lcm_ != nullptr) ;
		init_param(lcm_, keep_updated) ;
    }

	void init_param(lcm::LCM * lcm, bool keep_updated)
	{
		lcm_t * lcm_obj = lcm->getUnderlyingLCM() ;
        while(true) {
            param_ = bot_param_new_from_server(lcm_obj, keep_updated);
            if (!param_)
                std::cout << std::endl << "nuparam_t cannot find a param-server." << std::endl;
            else
                return;
            constexpr double retry_duration = 0.1;
            std::cout << "retrying after ~" << retry_duration << "s." << std::endl;
            usleep(1e5);
        }
	}

    // Prevent accidental copying of param_t
    param_t(const param_t&) = delete;
    param_t& operator=(const param_t&) = delete;

    BotParam* getBotParam() const
    {
        return param_;
    }

	lcm::LCM* getLCM() const 
	{
		return lcm_ ;
	}
	
public:
    /**
     * return true if constructor successfully created BotParam object for query
     */
    // name was chosen similar to lcm_t::isGood()
    bool isGood() const {
        return param_ != nullptr;
    }

    /**
     * if a non-empty namespace is set, each param_name will be formatted as
     *  namespace + "." + param_name
     */
    // TODO redefine API as pushNamespace(string);
    void setNamespace(const std::string& namespace_value) {
        ns_ = namespace_value;
    }

    // TODO add function to API as popNamespace();

    // TODO make function getNamespace() private
public:
    /**
     * return namespace value
     */
    std::string getNamespace() const {
        return ns_;
    }

private:
    /**
     * get full name of parameter either
     * 1) namespace.param_name
     * 2) param_name
     */
    std::string getFullName(const std::string& param_name) const {
        return ns_.empty() ? param_name : ns_ + "." + param_name;
    }

public:
    /**
     * gets parameter (if present) from parameter server.
     * returns default value if not present in parameter server.
     */
    template<typename type>
    type getParam(const std::string& param_name, const type& default_val) const {
        type param_val;
        getParam(param_name, param_val, default_val);
        return param_val;
    }

public:
    /**
     * gets parameter (if present) from parameter server.
     * returns default value if not present in parameter server.
     *
     * @return true if mapping contains key param_name
     */
    template<typename type>
    bool getParam(const std::string& param_name, type& param_val, const type& default_val) const {
        std::string full_param_name = getFullName(param_name);
        bool success = hasKey(full_param_name);
        if (success) {
            getParamHelper(full_param_name, param_val);
        } else {
            fprintf(stderr,
                    "BotParam: Key \"%s\" undefined; use default \"%s\"\n",
                    full_param_name.c_str(),
                    GenericFormat::toString(default_val).c_str());
            param_val = default_val;
        }
        return success;
    }

public:
    /**
     * return File object either
     *  1) based on parameter defined if key is present
     *  2) File::fromShell(default_value) otherwise
     */
    had::File getFile(const std::string& key, const had::File& default_file) const {
        std::string value;
        bool success = getParam(key, value, default_file.toString());
        return success ? had::File::fromShell(value) : default_file;
    }

public:
    /** Gets list of parameters of same type (if present) from parameters server.
     *  Returns default value for each parameter if not present in parameter server.
     *  Guaranteed to initialize all parameters to either read value or default value.
     * @param[in] param_names
     * @param[out] param_vals
     * @param[in] default_vals
     * @return 0 on success, -1 on failure (i.e., cannot read param)
     */
    template<typename type>
    int getParams(const std::vector<std::string>& param_names,
                  std::vector<type>& param_vals,
                  const std::vector<type>& default_vals) const {
        nu_assert(param_names.size() == default_vals.size());

        param_vals.clear();
        param_vals = default_vals;

        for (size_t idx = 0; idx != param_names.size(); ++idx) {
            if (!getParam(param_names.at(idx), param_vals.at(idx), default_vals.at(idx)))
                return -1;
        }

        return 0;
    }

public:
    /**
     * Checks if key is present in parameter server.
     *
     * returns true if the key is present, false if not
     */
    bool hasKey(const std::string& key) const {
        return bot_param_has_key(param_, key.c_str()) == 1;
    }

private:
    //data-type: "string"
    // function does not expand_relative_path(value);
    // returns 0 on success, -1, on failure
    int getParamHelper(const std::string& key, std::string& value) const {
        char* keyval;
        int ret = bot_param_get_str(param_, key.c_str(), &keyval);
        // only assign value is return value is "success"
        if (ret == 0) {
            value = std::string(keyval);
            // "free" needed based on bot_param_get_str API
            free(keyval);
        }
        return ret;
    }

private:
    //data-type: "int"
    // returns 0 on success, -1, on failure
    int getParamHelper(const std::string& key, int& value) const {
        return bot_param_get_int(param_, key.c_str(), &value);
    }

private:
    //data-type: "double"
    // returns 0 on success, -1, on failure
    int getParamHelper(const std::string& key, double& value) const {
        return bot_param_get_double(param_, key.c_str(), &value);
    }

private:
    //data-type: "bool"
    // returns 0 on success, -1, on failure
    int getParamHelper(const std::string& key, bool& value) const {
        int keyvalue;
        int ret = bot_param_get_boolean(param_, key.c_str(), &keyvalue);
        value = keyvalue ? true : false;
        return ret;
    }

public:
    /**
     * returns string representation of configuration
     *
     * the string lists each key value pair on a separate line.
     * a line is of the following form:
     *  key = "value";
     */
    std::string toString() {
        char* bytes;
        bot_param_write_to_string(param_, &bytes);
        std::string string(bytes);
        std::free(bytes);
        return string;
    }

public:

    /**
     * @brief Pair returns
     * @param key
     * @return first: true for success; second: string value upon success
     */

    std::pair<bool, std::string> getString(const std::string& key) const
    {
        std::pair<bool, std::string> result;

        const std::string full_name = getFullName(key);
        if(hasKey(full_name)) {
            int ret = getParamHelper(full_name, result.second);
            result.first = (ret==0)? true : false; // success is 0 !!!
        }
        else result.first = false;

        return result;
    }

    /// vector size is 0 upon failure
    std::vector<std::string> getStringVector(const std::string& key) const
    {
        std::vector<std::string> result;
        const std::string full_name = getFullName(key);
        int vector_size = bot_param_get_array_len(param_, full_name.c_str());

        if(vector_size>0){
            char** raw_data = bot_param_get_str_array_alloc(param_, full_name.c_str());

            result.resize(vector_size);
            for(int i=0; i<vector_size; ++i) {
                result[i] = raw_data[i];
            }
            bot_param_str_array_free(raw_data);
        }

        return result;
    }

    /// vector size is 0 upon failure
    std::vector<double> getDoubleArray(const std::string& key) const
    {
        std::vector<double> result;
        getArray(key, result);
        return result;
    }

    /// vector size is 0 upon failure
    std::vector<int> getIntArray(const std::string& key) const
    {
        std::vector<int> result;
        getArray(key, result);
        return result;
    }

private:

    template<class Type>
    void getArray(const std::string& key, std::vector<Type>& result) const
    {
        const std::string full_name = getFullName(key);
        int vector_size = bot_param_get_array_len(param_, full_name.c_str());

        if(vector_size>0){
            Type raw_data[vector_size];

            if(getArrayHelper(full_name, raw_data, vector_size)!=vector_size)
                return;

            result.resize(vector_size);
            for(int i=0; i<vector_size; ++i) {
                result[i] = raw_data[i];
            }
        }
    }

    int getArrayHelper(const std::string& key, double value[], int len) const
    {
        return bot_param_get_double_array(param_, key.c_str(), value, len);
    }

    int getArrayHelper(const std::string& key, int value[], int len) const
    {
        return bot_param_get_int_array(param_, key.c_str(), value, len);
    }
} ;

#endif
