#include <testing_framework/core/cli.h>
#include <testing_framework/core/commands.h>
#include <testing_framework/core/cmdline_parser.h>
#include <iostream>
#include <regex>

namespace testing
{

    struct UserParam
    {
        const std::type_info*type;
        std::function<bool(std::string)> validator;
        std::string short_name, long_name;
        std::string default_value;
    };

    std::map<std::string, UserParam> user_params;

    static std::vector<std::string> get_test_flags()
    {
        std::vector<std::string> fs;
        for (const auto&[name, p] : user_params)
        {
            if (p.type == nullptr)
            {
                fs.push_back(p.short_name);
                fs.push_back(p.long_name);
            }
        }
        return fs;
    }

    static std::vector<std::string> get_test_params()
    {
        std::vector<std::string> ps;
        for (const auto&[name, p] : user_params)
        {
            if (p.type != nullptr)
            {
                ps.push_back(p.short_name);
                ps.push_back(p.long_name);
            }
        }
        return ps;
    }

    static std::string skipped_options[] = {
        "-c",
        "-nc"
    };

    static std::string flag_names[] = {
        "-c",
        "-nc",
        "-d",
        "-R"
    };

    static std::string param_names[] = {
        "-r",
        "-l",
        "-j"
    };

    static std::vector<std::string> get_all_flags()
    {
        auto fs = get_test_flags();
        std::copy(std::begin(flag_names), std::end(flag_names), std::back_inserter(fs));
        return fs;
    }

    static std::vector<std::string> get_all_params()
    {
        auto ps = get_test_params();
        std::copy(std::begin(param_names), std::end(param_names), std::back_inserter(ps));
        return ps;
    }

    bool collect_test_by_name(std::string_view name, std::vector<const Test*>&out)
    {
        for (const Test*i : Test::all())
        {
            if (i->name() == name)
            {
                out.push_back(i);
                return true;
            }
        }
        std::cerr << "Test with name '" << name << "' does not exist." << std::endl;
        return false;
    }

    bool collect_tests_by_regex(std::string_view text, std::vector<const Test*>&out)
    {
        try
        {
            std::regex reg{std::string{text}};
            bool matched_any = false;
            for (const Test*i : Test::all())
            {
                if (std::regex_match(std::string(i->name()), reg))
                {
                    out.push_back(i);
                    matched_any = true;
                }
            }
            if (!matched_any)
            {
                std::cerr << "No test name mached regular expression '" << text << "'." << std::endl;
            }
            return matched_any;
        }
        catch (const std::regex_error&e)
        {
            std::cerr << "Invaid regular expression: " << e.what() << "." << std::endl;
            return false;
        }
    }


    bool collect_user_flag(std::string flag, std::map<std::string, std::pair<const std::type_info*, std::string>>&out)
    {
        for (const auto&[name, p] : user_params)
        {
            if (flag == p.short_name || flag == p.long_name)
            {
                out[name] = {nullptr, ""};
                return true;
            }
        }
        return false;
    }

    bool collect_user_param(
        std::string param,
        std::string value,
        std::map<std::string, std::pair<const std::type_info*, std::string>>&out,
        bool&valid
    )
    {
        for (const auto&[name, p] : user_params)
        {
            if (param == p.short_name || param == p.long_name)
            {
                valid = p.validator(value);
                out[name] = {p.type, value};
                return true;
            }
        }
        return false;
    }

    void fill_default_user_params(std::map<std::string, std::pair<const std::type_info*, std::string>>&out)
    {
        for (const auto&[name, p] : user_params)
        {
            auto it = out.find(name);
            if (it == out.end())
            {
                out[name] = {p.type, p.default_value};
            }
        }
    }

    bool handle_help(size_t argc, char**argv)
    {
        return true;
    }
    
    bool handle_list(size_t argc, char**argv)
    {
        std::vector<const Test*> tests;
        bool show_descriptions = false;

        if (!parse_args(
            argc,
            argv,
            skipped_options,
            get_all_flags(),
            get_all_params(),
            [&](std::string_view flag)->bool{
                if (flag == "-d")
                {
                    show_descriptions = true;
                    return true;
                }
                else
                {
                    std::cerr << "Flag '" << flag << "' is not applicable to list command." << std::endl;
                    return false;
                }
            }, 
            [&](std::string_view name, std::string_view value)->bool{
                if (name == "-r")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else
                {
                    std::cerr << "Param '" << name << "' is not applicable to list command." << std::endl;
                    return false;
                }
            },
            [&](std::string_view other)->bool{
                return collect_test_by_name(other, tests);
            }
        ))
        {
            return false;
        }

        //return list(tests, show_descriptions);
        return true;
    }

    bool handle_run(size_t argc, char**argv)
    {
        std::vector<const Test*> tests;
        std::map<std::string, std::pair<const std::type_info*, std::string>> user_params;
        size_t logging_level = 0;
        size_t jobs = 1;

        if (!parse_args(
            argc,
            argv,
            skipped_options,
            get_all_flags(),
            get_all_params(),
            [&](std::string_view flag)->bool{
                if (collect_user_flag(std::string(flag), user_params)) {
                    return true;
                } else {
                    std::cerr << "Flag '" << flag << "' is not applicable to run command." << std::endl;
                    return false;
                }
            }, 
            [&](std::string_view name, std::string_view value)->bool{
                bool valid  = false;
                if (name == "-l")
                {
                    // validate somehow
                    return true;
                }
                else if (name == "-j")
                {
                    // validate somehow
                    return true;
                }
                else if(name == "-r")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else if(collect_user_param(std::string(name), std::string(value), user_params, valid))
                {
                    return valid;
                }
                else
                {
                    std::cerr << "Param '" << name << "' is not applicable to run command." << std::endl;
                    return false;
                }
            },
            [&](std::string_view other)->bool{
                return collect_test_by_name(other, tests);
            }
        ))
        {
            return false;
        }
        fill_default_user_params(user_params);
        //return run(logging_level, jobs, tests, user_params);
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