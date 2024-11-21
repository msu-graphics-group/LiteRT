#include <testing_framework/core/cli.h>
#include <testing_framework/core/commands.h>
#include <testing_framework/core/cmdline_parser.h>
#include <testing_framework/core/test_options.h>
#include <testing_framework/core/exe.h>
#include <testing_framework/core/colors.h>
#include <iostream>
#include <regex>

namespace testing
{
    constexpr size_t DEFAULT_LOGGING_LEVEL = 10;
    constexpr size_t DEFAULT_JOBS = 1;

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
        int64_t logging_level = DEFAULT_LOGGING_LEVEL;
        int64_t jobs = DEFAULT_JOBS;

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
        return run(logging_level, jobs, tests, test_options);
    }

    bool handle_exec(size_t argc, char**argv)
    {
        std::vector<const Test*> tests;
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options;
        int64_t logging_level = DEFAULT_LOGGING_LEVEL;
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
        return exec(logging_level, rewrite, tests[0], test_options);
    }

    bool handle_args(size_t argc, char**argv)
    {
        
        if (argc < 2)
        {
            return false;
        }
        argc--;
        argv++;

        for (size_t i = 0; i < argc; i++)
        {
            std::string_view arg = argv[i];
            if (arg == "-c")
            {
                set_colors_enabled(true);
            }
            else if (arg == "-nc")
            {
                set_colors_enabled(false);
            }
        }

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

    bool yes_or_no_dialogue(bool&result)
    {
        static std::string yes[] = {"yes", "Y", "y"};
        static std::string no[] = {"no", "N", "n"};
        
        while (true)
        {
            std::cout << "Please, answer [Y/N]";
            std::string line;
            std::getline(std::cin, line);
            if (std::cin.eof())
            {
                std::cout << std::endl;
                return false;
            }
            if (line.size() > 0 && line.back() == '\n')
            {
                line.resize(line.size() - 1);
            }
            auto y = std::find(std::begin(yes), std::end(yes), line);
            auto n = std::find(std::begin(no), std::end(no), line);

            if (y != std::end(yes))
            {
                result = true;
                return true;
            }
            else if(n != std::end(no))
            {
                result = false;
                return true;
            }

        }
    }

    std::vector<std::string> cmdline_to_exec(
        bool enable_colors,
        size_t logging_level,
        std::string test_name,
        bool rewrite,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    )
    {
        std::vector<std::string> out;
        out.push_back(current_executable_path().string());
        out.push_back("exec");
        if (enable_colors)
        {
            out.push_back("-c");
        }
        else
        {
            out.push_back("-nc");
        }
        out.push_back("-l");
        out.push_back(std::to_string(logging_level));
        out.push_back(test_name);
        for (const auto& [name, p] : test_options)
        {
            const auto&[type, value] = p;
            if (type == nullptr)
            {
                if (value.length() > 0)
                {
                    out.push_back(get_test_option_short_name(name));
                }
            }
            else
            {
                out.push_back(get_test_option_short_name(name));
                out.push_back(value);
            }
        }       
        return out;
    }

}