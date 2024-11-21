#include <testing_framework/core/cli.h>
#include <testing_framework/core/commands.h>
#include <testing_framework/core/cmdline_parser.h>
#include <testing_framework/core/test_options.h>
#include <iostream>
#include <regex>

namespace testing
{

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
        auto fs = get_test_flag_names();
        std::copy(std::begin(flag_names), std::end(flag_names), std::back_inserter(fs));
        return fs;
    }

    static std::vector<std::string> get_all_params()
    {
        auto ps = get_test_param_names();
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
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options;
        int64_t logging_level = 0;
        int64_t jobs = 1;

        if (!parse_args(
            argc,
            argv,
            skipped_options,
            get_all_flags(),
            get_all_params(),
            [&](std::string_view flag)->bool{
                if (collect_test_flag(std::string(flag), test_options)) {
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
                    return validate_param(std::string{name}, std::string{value}, logging_level)
                        && validate_is_non_negative_param(std::string{name}, logging_level);
                }
                else if (name == "-j")
                {
                    return validate_param(std::string{name}, std::string{value}, jobs)
                        && validate_is_positive_param(std::string{name}, jobs);
                }
                else if(name == "-r")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else if(collect_test_param(std::string(name), std::string(value), test_options, valid))
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
        collect_default_test_param_values(test_options);
        //return run(logging_level, jobs, tests, test_options);
        return true;
    }

    bool handle_exec(size_t argc, char**argv)
    {
        std::vector<const Test*> tests;
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options;
        int64_t logging_level = 0;
        bool rewrite = false;

        if (!parse_args(
            argc,
            argv,
            skipped_options,
            get_all_flags(),
            get_all_params(),
            [&](std::string_view flag)->bool{
                if (flag == "-R")
                {
                    rewrite = true;
                    return true;
                }
                else if (collect_test_flag(std::string(flag), test_options)) {
                    return true;
                } else {
                    std::cerr << "Flag '" << flag << "' is not applicable to exec command." << std::endl;
                    return false;
                }
            }, 
            [&](std::string_view name, std::string_view value)->bool{
                bool valid  = false;
                if (name == "-l")
                {
                    return validate_param(std::string{name}, std::string{value}, logging_level)
                        && validate_is_non_negative_param(std::string{name}, logging_level);
                }
                else if(name == "-r")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else if(collect_test_param(std::string(name), std::string(value), test_options, valid))
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
        collect_default_test_param_values(test_options);

        if (tests.size() == 0)
        {
            std::cerr << "test must be specified for exec command" << std::endl;
            return false;
        }
        else if(tests.size() > 1)
        {
            std::cerr << "exec command accepts only one test, but many where specified" << std::endl;
        }
        // return exec(logging_level, rewrite, tests[0], test_options);
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
        else if(cmd == "exec")
        {
            return handle_exec(argc, argv);
        }
        else
        {
            std::cerr << "Bad command" << std::endl;
            return false;
        }

    }

}