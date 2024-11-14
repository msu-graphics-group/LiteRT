#include "args.h"
#include <iostream>
#include "help.h"

namespace test
{

    bool Args::is_arg_name(std::string_view x)
    {
        if (x.size() <= 2)
            return false;
        if (x[0] != '-' || x[1] != '-')
            return false;
        for (size_t i = 2; i < x.size(); i++)
            if (!std::isalnum(x[i]))
                return false;
        return true;
    }

    std::optional<Args> Args::parse(size_t argc, char**argv)
    {
        Args out;
        std::string_view key;
        for (int i = 0; i < argc; i++)
        {
            std::string_view x = argv[i];
            if (is_arg_name(x)) {
                key = x;
                out.args_[key];
            } else {
                if (is_arg_name(key)) {
                    out.args_[key].push_back(x);
                } else {
                    // there was no key, which means is starts with no-name; aka bad case
                    std::cerr << "'" << x << "' is not an argument name." << std::endl;
                    std::cerr << "Argument names start from '--'." << std::endl;
                    std::cerr << read_help_message << std::endl;
                    return std::nullopt;
                }
            }
        }
        return out;
    }

    bool Args::check_only(const std::vector<std::string_view>&names) const
    {
        for (auto&[key, values] : args_)
        {
            auto it = std::find(names.begin(), names.end(), key);
            if (it != names.end())
            {
                std::cerr << "Unrecognized argument name '" << key << "'." << std::endl;
                std::cerr << read_help_message << std::endl;
                return false;
            }
        }
        return true;
    }

    bool Args::get(std::string_view name, bool&x) const
    {
        auto it = args_.find(name);
        if (it == args_.end())
        {
            x = false;
            return true;
        }
        if (it->second.size() == 0)
        {
            x = true;
            return true;
        }
        std::cerr << "'" << name << "' is a flag argument and can not have any values, but '" << it->second[0] <<"' was specified." << std::endl;
        std::cerr << read_help_message << std::endl;
        return false;
    }

    bool Args::get(std::string_view name, std::optional<std::string_view>&x) const
    {
        auto it = args_.find(name);
        if (it == args_.end())
        {
            x = std::nullopt;
            return true;
        }
        if (it->second.size() == 1)
        {
            x = it->second[0];
            return true;
        }
        std::cerr << "'" << name << "' is a value argument and must have one value assigned, but ";
        if (it->second.size() == 0)
        {
            std::cerr << "none was";
        }
        else
        {
            for (auto i : it->second)
            {
                std::cerr << "'" << i << "' ";
            }
            std::cerr << "were";
        }
        std::cerr << " specified." << std::endl;
        std::cerr << read_help_message << std::endl;
        return false;
    }
    
    bool Args::get(std::string_view name, std::optional<std::vector<std::string_view>>&x) const
    {
        auto it = args_.find(name);
        if (it == args_.end())
        {
            x = std::nullopt;
            return true;
        }
        x = it->second;
        return true;
    }

}