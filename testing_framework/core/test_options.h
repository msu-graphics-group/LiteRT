#pragma once
#include <string>
#include <map>
#include <functional>
#include <typeinfo>

namespace testing
{

    /*
        Adds option:
            Option can be accessed by <name>.
            It can be specified in cmdline be <short_name> or <long_name>.
            When option is specfied it's value is validated using
                validator((<parsed_name>, <value>));
            If <type> is nullptr, then option is considered a flag.
            Otherwise <type> must be equal to a type <value> of option must be parsed (int, float, std::string, etc.)
    */
    void add_test_option(
        std::string name,
        const std::type_info*type,
        std::function<bool(std::string, std::string)> validator,
        std::string short_name,
        std::string long_name,
        std::string default_value
    );

    inline void add_test_flag(
        std::string name,           // "flag"
        std::string short_name,     // "-f"
        std::string long_name       // --flag
    )
    {
        add_test_option(name, nullptr, {}, short_name, long_name, "");
    }
    
    template<typename T, typename CustomValidator>
    void add_test_param(
        std::string name,
        std::string short_name,
        std::string long_name,
        std::string default_value,
        CustomValidator&&custom_validator = [](std::string&, const T&)->bool{ return true; }
    )
    {
        add_test_option(
            name,
            &typeid(T),
            [custom_validator = std::forward<CustomValidator>(custom_validator)]
            (std::string name, std::string value)
            {
                T tmp;
                if (!validate_param(name, value, tmp))
                {
                    return false;
                }
                return custom_validator(name, tmp);
            },
            short_name,
            long_name,
            default_value
        );
    }

    std::vector<std::string> get_test_flag_names();
    std::vector<std::string> get_test_param_names();

    std::string get_test_option_short_name(std::string name);

    /*
        If <name> is flag name, adds option in <out> and returns true
        Otherwise returns false
    */
    bool collect_test_flag(
        std::string name, 
        std::map<std::string, std::pair<const std::type_info*, std::string>>&out
    );

    /*
        If <name> is not a param, then returns true
        Otherwise validates <value> and if <value> is valid adds option in <out>,
            returns validation result in <valid>; returns true
    */
    bool collect_test_param(
        std::string name,
        std::string value,
        std::map<std::string, std::pair<const std::type_info*, std::string>>&out,
        bool&valid
    );

    /*
        If there is no <value> for <param> in <out> then adds <param> with <default_value>
    */
    void collect_default_test_param_values(std::map<std::string, std::pair<const std::type_info*, std::string>>&out);

}