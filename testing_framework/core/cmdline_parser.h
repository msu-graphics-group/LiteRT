#pragma once
#include <string>
#include <array>
#include <iostream>

namespace testing
{

    template<
        typename IsFlagName,
        typename IsParamName,
        typename FlagHandler,
        typename ParamHandler,
        typename OtherHandler
    >
    static bool generic_parse_arg(
        size_t argc,
        char** argv,
        size_t&offset,
        IsFlagName &&is_flag_name,
        IsParamName &&is_param_name,
        FlagHandler &&flag,
        ParamHandler &&param,
        OtherHandler &&other
    )
    {
        if (argc - offset == 0)
        {
            return true;
        }
        std::string_view arg = argv[offset];
        if (arg.length() >= 2 && arg[0] == '-')
        {
            if (is_flag_name(arg))
            {
                offset++;
                return flag(arg);
            }
            for (size_t len = 2; len <= arg.length(); len++)
            {
                std::string_view cut(arg.begin(), arg.begin() + len);
                if (is_param_name(cut))
                {
                    offset++;
                    if (cut.length() < arg.length()) // value is part of arg
                    {
                        return param(cut, std::string_view(arg.begin() + len, arg.end()));
                    }
                    else // value is next arg
                    {
                        if (argc - offset == 0)
                        {
                            std::cerr << "Expected value for '" << cut << "'" << std::endl;
                            return false;
                        }
                        else
                        {
                            offset++;
                            return param(cut, argv[offset-1]);
                        }
                    }

                }
            }
            offset++;
            std::cerr << "'" << arg << "' is not a recognised option" << std::endl;
            // failed to parse, but ok
            return false;
        }
        else
        {
            offset++;
            return other(arg);
        }
    }
    
    template<
        typename SkipOption,
        typename IsFlagName,
        typename IsParamName,
        typename FlagHandler,
        typename ParamHandler,
        typename OtherHandler
    >
    static bool generic_parse_args(
        size_t argc,
        char**argv,
        SkipOption &&skip_option,
        IsFlagName &&is_flag_name,
        IsParamName &&is_param_name,
        FlagHandler &&flag,
        ParamHandler &&param,
        OtherHandler &&other
    )
    {
        bool wrong_input = false;

        for (size_t offset = 0;offset<argc;)
        {
            wrong_input |= !generic_parse_arg(
                argc,
                argv,
                offset,
                is_flag_name,
                is_param_name,
                [&](std::string_view name)->bool{
                    if (!wrong_input && !skip_option(name))
                    {
                        return flag(name);
                    }
                    return true;
                },
                [&](std::string_view name, std::string_view value)->bool{
                    if (!wrong_input && !skip_option(name))
                    {
                        return param(name, value);
                    }
                    return true;
                },
                [&](std::string_view value)->bool{
                    if (!wrong_input)
                    {
                        return other(value);
                    }
                    return true;
                }
            );
        }
        return !wrong_input;
    }

    template<
        typename Skipped,
        typename FlagNames,
        typename ParamNames,
        typename FlagHandler,
        typename ParamHandler,
        typename OtherHandler
    >
    static bool parse_args(
        size_t argc,
        char **argv,
        Skipped&&skipped,
        FlagNames&&flags,
        ParamNames&&params,
        FlagHandler flag,
        ParamHandler param,
        OtherHandler other
    )
    {
        auto is_skipped = [&](std::string_view x){
            return std::find(std::begin(skipped), std::end(skipped), x) != std::end(skipped);
        };
        auto is_flag = [&](std::string_view x){
            return std::find(std::begin(flags), std::end(flags), x) != std::end(flags);
        };
        auto is_param = [&](std::string_view x){
            return std::find(std::begin(params), std::end(params), x) != std::end(params);
        };
        return generic_parse_args(
            argc,
            argv,
            is_skipped,
            is_flag,
            is_param,
            std::forward<FlagHandler>(flag),
            std::forward<ParamHandler>(param),
            std::forward<OtherHandler>(other)
        );
    }

}