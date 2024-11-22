#include <testing_framework/core/test_options.h>

namespace testing
{

    struct TestOption
    {
        std::string name;
        const std::type_info*type;
        std::function<bool(std::string_view, std::string_view)> validator;
        std::string short_name, long_name;
        std::string default_value;
        std::string description;
    };

    static std::vector<TestOption> registered_options;


    void add_test_option(
        std::string name,
        const std::type_info*type,
        std::function<bool(std::string_view, std::string_view)> validator,
        std::string short_name,
        std::string long_name,
        std::string default_value,
        std::string description
    )
    {
        registered_options.push_back(
            {
                std::move(name),
                type,
                validator,
                std::move(short_name),
                std::move(long_name),
                std::move(default_value),
                std::move(description)
            }
        );
    }

    std::vector<std::string> get_test_flag_names()
    {
        std::vector<std::string> names;
        for (const auto&opt : registered_options)
        {
            if (opt.type == nullptr)
            {
                names.push_back(opt.short_name);
                names.push_back(opt.long_name);
            }
        }
        return names;
    }
    std::vector<std::string> get_test_param_names()
    {
        std::vector<std::string> names;
        for (const auto&opt : registered_options)
        {
            if (opt.type != nullptr)
            {
                names.push_back(opt.short_name);
                names.push_back(opt.long_name);
            }
        }
        return names;
    }

    std::string get_test_option_short_name(std::string_view name)
    {
        for (const auto&opt : registered_options)
        {
            if (opt.name == name)
            {
                return opt.short_name;
            }
        }
        return "";
    }

    std::vector<std::tuple<std::string, std::string, std::string>> get_options_info()
    {
        std::vector<std::tuple<std::string, std::string, std::string>> out;
        for (const auto&opt : registered_options)
        {
            out.push_back({opt.short_name, opt.long_name, opt.description});
        }
        return out;
    }

    bool collect_test_flag(
        std::string_view name, 
        std::map<std::string, std::pair<const std::type_info*, std::string>>&out
    )
    {
        for (const auto&opt : registered_options)
        {
            if (opt.type == nullptr && (name == opt.short_name || name == opt.long_name))
            {
                out[opt.name] = {nullptr, "1"};
                return true;
            }
        }
        return false;
    }

    bool collect_test_param(
        std::string_view name,
        std::string_view value,
        std::map<std::string, std::pair<const std::type_info*, std::string>>&out,
        bool&valid
    )
    {
        for (const auto&opt : registered_options)
        {
            if (opt.type != nullptr && (name == opt.short_name || name == opt.long_name))
            {
                valid = opt.validator(name, value);
                if (valid)
                {
                    out[opt.name] = {opt.type, std::string(value)};
                }
                return true;
            }
        }
        return false;
    }

    void collect_default_test_param_values(std::map<std::string, std::pair<const std::type_info*, std::string>>&out)
    {
        for (const auto&opt : registered_options)
        {
            auto it = out.find(opt.name);
            if (it == out.end())
            {
                if (opt.type == nullptr)
                {
                    out[opt.name] = {nullptr, ""};
                }
                else
                {
                    out[opt.name] = {opt.type, opt.default_value};
                }
            }
        }
    }

}