#include <testing_framework/core/cli.h>
#include <testing_framework/core/commands.h>
#include <iostream>

namespace testing
{

    constexpr size_t MAX_OPTION_NAME_LENGTH = 2;

    /*
        For each cmdline arguments checks
            - if it is a flag
            - if it is a pararameter
        If argument "-<name>"" is flag or parameter and
            if should_skip(<name>) returns true then arguement is skipped
    */

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
            std::string_view assumed_name(arg.begin() + 1, arg.end());
            if (is_flag_name(assumed_name))
            {
                offset++;
                return flag(arg);
            }
            for (size_t len = 1; len <= std::max(MAX_OPTION_NAME_LENGTH, assumed_name.length()); len++)
            {
                std::string_view cut(assumed_name.begin(), assumed_name.begin() + len);
                if (is_param_name(cut))
                {
                    offset++;
                    if (cut.length() < assumed_name.length()) // value is part of arg
                    {
                        return param(cut, std::string_view(assumed_name.begin() + len, assumed_name.end()));
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
        typename ShouldSkip,
        typename IsFlagName,
        typename IsParamName,
        typename FlagHandler,
        typename ParamHandler,
        typename OtherHandler
    >
    static bool generic_parse_args(
        size_t argc,
        char**argv,
        ShouldSkip &&should_skip,
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
            wrong_input &= generic_parse_arg(
                argc,
                argv,
                offset,
                is_flag_name,
                is_param_name,
                [&](std::string_view name)->bool{
                    if (!wrong_input && !should_skip(name))
                    {
                        return flag(name);
                    }
                    return true;
                },
                [&](std::string_view name, std::string_view value)->bool{
                    if (!wrong_input && !should_skip(name))
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
        typename FlagHandler,
        typename ParamHandler,
        typename OtherHandler
    >
    static bool parse_args(
        size_t argc,
        char **argv,
        FlagHandler&& flag,
        ParamHandler&& param,
        OtherHandler&& other
    )
    {
        static std::string_view skipped[] = {
            "c", "nc"
        };
        static std::string_view flags[] = {
            "c", "nc", "d"
        };
        static std::string_view params[] = {
            "r"
        };
        static auto is_skipped = [](std::string_view x){
            return std::find(std::begin(skipped), std::end(skipped), x) != std::end(skipped);
        };
        static auto is_flag = [](std::string_view x){
            return std::find(std::begin(flags), std::end(flags), x) != std::end(flags);
        };
        static auto is_param = [](std::string_view x){
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

    const Test* find_test_by_name(std::string_view);
    std::vector<const Test*> find_test_by_regex(std::string_view);

    bool handle_help(size_t argc, char**argv)
    {
        return true;
    }
    
    bool handle_list(size_t argc, char**argv)
    {
        std::vector<const Test*> named;
        std::vector<const Test*> regs;
        bool show_descriptions = false;

        if (!parse_args(
            argc,
            argv,
            [&](std::string_view flag)->bool{
                if (flag == "d")
                {
                    show_descriptions = true;
                    return true;
                }
                else
                {
                    return false;
                }
            }, 
            [&](std::string_view param_name, std::string_view param_value)->bool{
                return false;
            },
            [&](std::string_view other)->bool{
                return false;
            }
        ))
        {
            return false;
        }

        return true;
    }

    bool handle_run(size_t argc, char**argv)
    {
        return true;
    }

    bool handle_rewrite(size_t argc, char**argv)
    {
        return true;
    }

    bool handle_unsafe(size_t argc, char**argv)
    {
        return true;
    }

    bool handle_args(size_t argc, char**argv)
    {
        
        if (argc < 2)
        {
            return false;
        }
        argc--;
        argv++;

        std::string_view cmd = argv[0];
        
        argc--;
        argv++;

        if (cmd == "help")
        {
            return handle_help(argc, argv);
        }
        else if(cmd == "list")
        {
            return handle_list(argc, argv);
        }
        else if(cmd == "run")
        {
            return handle_run(argc, argv);
        }
        else if(cmd == "rewrite")
        {
            return handle_rewrite(argc, argv);
        }
        else if(cmd == "unsafe")
        {
            return handle_unsafe(argc, argv);
        }
        else
        {
            std::cerr << "Bad command" << std::endl;
            return false;
        }

    }

}