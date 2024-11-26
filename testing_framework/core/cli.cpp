#include <testing_framework/core/cli.h>
#include <testing_framework/core/commands.h>
#include <testing_framework/core/cmdline_parser.h>
#include <testing_framework/core/test_options.h>
#include <testing_framework/core/param_validators.h>
#include <testing_framework/core/exe.h>
#include <testing_framework/core/colors.h>
#include <testing_framework/core/exe.h>
#include <iostream>
#include <regex>
#include <fstream>

namespace testing
{
    constexpr size_t DEFAULT_LOGGING_LEVEL = 10;
    constexpr size_t DEFAULT_JOBS = 1;

    static std::string skipped_options[] = {
        "-c", "--colors",
        "-nc", "--no-colors"
    };

    static std::string flag_names[] = {
        "-c", "--colors",
        "-nc", "--no-colors",
        "-d", "--description",
        "-R", "--rewrite",
        "-nf", "--no-filter"
    };

    static std::string param_names[] = {
        "-r", "--regex",
        "-l", "--logging-level",
        "-j", "--jobs"
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
        std::cerr << foreground(error_color) << "Error: " << default_color
            << "no test with name " << foreground(highlight_color_1) << "'" << name << "'" << default_color << "." << std::endl;
        std::cerr << "See '" << current_executable_name().string() << " list' for all tests." << std::endl;
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
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "no test matched regular expression "
                    << foreground(highlight_color_1) << "'" << text << "'" << default_color << "." << std::endl;
                std::cerr << "See '" << current_executable_name().string() << " list' for all tests." << std::endl;
            }
            return matched_any;
        }
        catch (const std::regex_error&e)
        {
            
            std::cerr << foreground(error_color) << "Error: " << default_color
                << "invalid regular expression " 
                << foreground(highlight_color_1) << "'" << text << "'" << default_color
                << ": " << e.what() << "." << std::endl;
            return false;
        }
    }

    bool handle_help(size_t argc, char**argv)
    {

        std::string padding = "    ";

        auto print_opt = [&](
                std::string_view short_name,
                std::string_view long_name,
                std::string_view description,
                std::string_view offset
            ){
                std::cout << offset << foreground(option_color) << long_name << default_color
                    << description << "." << std::endl;
                if (short_name != long_name && short_name != "") {
                    std::cout << offset << foreground(option_color) << short_name << default_color
                        << " - alias for "
                        << foreground(option_color) << long_name << "" << default_color
                        << "." << std::endl;
                }
            };

        auto print_cmd = [&](
            std::string_view cmd,
            std::string_view args,
            std::string_view desciption,
            std::string_view offset
            ){
                std::cout << offset << foreground(command_color) << cmd << default_color
                << foreground(option_color) << args << default_color
                << " - " << desciption << std::endl;
            };
        
        std::cout << "General options:" << std::endl;
        print_opt("-c", "--colors", " - enable colors", padding);
        print_opt("-nc", "--no-colors", " - disable colors", padding);
        std::cout << padding << padding << "(colors are "
        << foreground(warning_color) << (get_colors_enabled_default() ? "enabled" : "disabled") << default_color
        << " by default)" << std::endl;
        std::cout << std::endl;

        std::cout << "Commands:" << std::endl;

        print_cmd("help", "", "print this text.", padding);
        std::cout << std::endl;

        print_cmd("list", " <tests> [-d]", "prints tests' names. If no tests are specified, assuming all tests.", padding);
        print_opt("-d", "--description", " - print descriptions of tests", padding + padding);
        std::cout << std::endl;

        print_cmd("run", " <tests> [-l] [-j] [-nf] <test-options>", "runs specified tests in supervised mode.", padding);
        std::cout << padding << padding << padding
            << "Each test is runned in seperate process, which lets handle crashes and runtime errors." << std::endl;
        std::cout << padding << padding << padding << "If no tests are specified, assuming all tests." << std::endl;
        print_opt("-l", "--logging-level", " <value> - only messages with logging level <value> and below are printed", padding + padding);
        print_opt("-j", "--jobs", " <value> - how many tests can be runned parallel", padding + padding);
        print_opt("-nf", "--no-filter", "- disables filtering of test's output", padding + padding);
        std::cout << std::endl;

        print_cmd("exec", " <test> [-l] [-R] <test-options>", " runs speicified test.", padding);
        print_opt("-R", "--rewrite", " - rewrites test's saved references", padding + padding);
        std::cout << std::endl;

        std::cout << "Tests list:" << std::endl;
        std::cout << padding << "Tests are specified by name or by regular expressions using following option:" << std::endl;
        print_opt("-r", "--regex", " <regexpr> - adds to <tests-list> all tests which are matched by <regex>", padding); 
        std::cout << std::endl;

        std::cout << "Test options:" << std::endl;
        std::cout << padding << "Following options' values can be accessed inside tests:" << std::endl;

        for (const auto&[short_name, long_name, description] : get_test_options_info())
        {
            print_opt(short_name, long_name, description, padding);
        }

        std::cout << std::endl;

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
                if (flag == "-d" || flag == "--description")
                {
                    show_descriptions = true;
                    return true;
                }
                else
                {
                    
                    std::cerr << foreground(error_color) << "Error: " << default_color
                        << "option " 
                        << foreground(option_color) << "'" << flag << "'" << default_color 
                        << " is not applicable to "
                        << foreground(command_color) << "list" << default_color
                        << " command." << std::endl;
                    return false;
                }
            }, 
            [&](std::string_view name, std::string_view value)->bool{
                if (name == "-r" || name == "--regex")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else
                {
                    std::cerr << foreground(error_color) << "Error: " << default_color
                        << "option " 
                        << foreground(option_color) << "'" << name << "'" << default_color 
                        << " is not applicable to "
                        << foreground(command_color) << "list" << default_color
                        << " command." << std::endl;
                    return false;
                }
            },
            [&](std::string_view other)->bool{
                return collect_test_by_name(other, tests);
            }
        ))
        {
            std::cerr << "See '" << current_executable_name().string() << " help'." << std::endl;
            return false;
        }

        return list(tests.size() > 0 ? tests : Test::all(), show_descriptions);
    }

    bool handle_run(size_t argc, char**argv)
    {
        std::vector<const Test*> tests;
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options;
        int64_t logging_level = DEFAULT_LOGGING_LEVEL;
        int64_t jobs = DEFAULT_JOBS;
        bool filter = true;

        if (!parse_args(
            argc,
            argv,
            skipped_options,
            get_all_flags(),
            get_all_params(),
            [&](std::string_view flag)->bool{
                if (flag == "-nf" || flag == "--no-filter")
                {
                    filter = false;
                    return true;
                }
                else if (collect_test_flag(std::string(flag), test_options)) {
                    return true;
                } else {
                    std::cerr << foreground(error_color) << "Error: " << default_color
                        << "option " 
                        << foreground(option_color) << "'" << flag << "'" << default_color 
                        << " is not applicable to "
                        << foreground(command_color) << "run" << default_color
                        << " command." << std::endl;
                    return false;
                }
            }, 
            [&](std::string_view name, std::string_view value)->bool{
                bool valid  = false;
                if (name == "-l" || name == "--logging-level")
                {
                    return validate_param(std::string{name}, std::string{value}, logging_level)
                        && validate_is_non_negative_param(std::string{name}, logging_level);
                }
                else if (name == "-j" || name == "--jobs")
                {
                    return validate_param(std::string{name}, std::string{value}, jobs)
                        && validate_is_positive_param(std::string{name}, jobs);
                }
                else if(name == "-r" || name == "--regex")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else if(collect_test_param(std::string(name), std::string(value), test_options, valid))
                {
                    return valid;
                }
                else
                {
                    std::cerr << foreground(error_color) << "Error: " << default_color
                        << "option " 
                        << foreground(option_color) << "'" << name << "'" << default_color 
                        << " is not applicable to "
                        << foreground(command_color) << "run" << default_color
                        << " command." << std::endl;
                    return false;
                }
            },
            [&](std::string_view other)->bool{
                return collect_test_by_name(other, tests);
            }
        ))
        {
            std::cerr << "See '" << current_executable_name().string() << " help'." << std::endl;
            return false;
        }
        collect_default_test_param_values(test_options);
        return run(logging_level, jobs, filter, tests.size() > 0 ? tests : Test::all(), test_options);
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
                if (flag == "-R" || flag == "--rewrite")
                {
                    rewrite = true;
                    return true;
                }
                else if (collect_test_flag(std::string(flag), test_options)) {
                    return true;
                } else {
                   std::cerr << foreground(error_color) << "Error: " << default_color
                        << "option " 
                        << foreground(option_color) << "'" << flag << "'" << default_color 
                        << " is not applicable to "
                        << foreground(command_color) << "exec" << default_color
                        << " command." << std::endl;
                    return false;
                }
            }, 
            [&](std::string_view name, std::string_view value)->bool{
                bool valid  = false;
                if (name == "-l" || name == "--logging-level")
                {
                    return validate_param(std::string{name}, std::string{value}, logging_level)
                        && validate_is_non_negative_param(std::string{name}, logging_level);
                }
                else if(name == "-r" || name == "--regex")
                {
                    return collect_tests_by_regex(value, tests);
                }
                else if(collect_test_param(std::string(name), std::string(value), test_options, valid))
                {
                    return valid;
                }
                else
                {
                    std::cerr << foreground(error_color) << "Error: " << default_color
                        << "option " 
                        << foreground(option_color) << "'" << name << "'" << default_color 
                        << " is not applicable to "
                        << foreground(command_color) << "exec" << default_color
                        << " command." << std::endl;
                    return false;
                }
            },
            [&](std::string_view other)->bool{
                return collect_test_by_name(other, tests);
            }
        ))
        {
            std::cerr << "See '" << current_executable_name().string() << " help'." << std::endl;
            return false;
        }
        collect_default_test_param_values(test_options);

        if (tests.size() == 0)
        {
            std::cerr << foreground(error_color) << "Error: " << default_color
                        << "test must be specified for "
                        << foreground(command_color) << "exec" << default_color
                        << " command." << std::endl;
            std::cerr << "See '" << current_executable_name().string() << " help'." << std::endl;
            return false;  
        }
        else if(tests.size() > 1)
        {
            std::cerr << foreground(error_color) << "Error: " << default_color
                        << "only one test can be specified for "
                        << foreground(command_color) << "exec" << default_color
                        << " command." << std::endl;
            std::cerr << "See '" << current_executable_name().string() << " help'." << std::endl;
            return false;  
        }
        return exec(logging_level, rewrite, tests[0], test_options);
    }

    bool handle_ctest(size_t argc, char**argv)
    {
        if (argc != 2)
        {
            std::cerr << foreground(error_color) << "Error: " << default_color
                << "creating ctest files requires to arguments: "
                << foreground(highlight_color_1) << "<ctest-include-file>" << " "
                << "<test-working-directory>" << default_color
                << std::endl;
            return false;
        }
        std::string ctest_include = argv[0];
        std::string working_dir = argv[1];
        std::ofstream file(ctest_include);
        if (!file)
        {
            std::cerr << foreground(error_color) << "Error: " << default_color 
                << "failed to open file "
                << foreground(highlight_color_2) << ctest_include << default_color
                << std::endl;
        }
        for (auto test : Test::all())
        {
            
            auto args = cmdline_to_exec(false, INFO_LOGGING_LEVEL, test->name(), false, {});
            file << "add_test(" << test->name() << " ";
            for (auto&i : args)
            {
                file << i << " ";
            }
            file << ")" << std::endl;
            file << "set_tests_properties(" << test->name() << " PROPERTIES WORKING_DIRECTORY " << working_dir << ")" << std::endl;
        }
        return true;
    }

    bool handle_args(size_t argc, char**argv)
    {
        
        if (argc < 2)
        {
            std::cerr << "Use '" << current_executable_name().string() << " run' to run tests." << std::endl;
            std::cerr << "See '" << current_executable_name().string() << " help' for more information." << std::endl;
            return false;
        }
        argc--;
        argv++;

        for (size_t i = 0; i < argc; i++)
        {
            std::string_view arg = argv[i];
            if (arg == "-c" || arg == "--colors")
            {
                set_colors_enabled(true);
            }
            else if (arg == "-nc" || arg == "--no-colors")
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
        else if (cmd == "generate-ctest-files")
        {
            return handle_ctest(argc, argv);
        }
        else
        {
            std::cerr << foreground(error_color) << "Error: " << default_color
                        << foreground(command_color)  << "'" << cmd << "'" << default_color
                        << " is not a recognised command." << std::endl;
            std::cerr << "See '" << current_executable_name().string() << " help'." << std::endl;
            return false;
        }

    }

    bool yes_or_no_dialogue(bool&result)
    {
        static std::string yes[] = {"yes", "Y", "y"};
        static std::string no[] = {"no", "N", "n"};
        
        while (true)
        {
            std::cout << "Please, answer [Y/N]:";
            std::string line;
            std::getline(std::cin, line);
            if (std::cin.eof())
            {
                std::cout << std::endl;
                std::cout << "Encountered end of input." << std::endl;
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
        out.push_back(std::move(test_name));
        for (auto& [name, p] : test_options)
        {
            auto&[type, value] = p;
            if (type == nullptr)
            {
                if (value.length() > 0)
                {
                    out.push_back(get_test_option_cli_name(name));
                }
            }
            else
            {
                out.push_back(get_test_option_cli_name(name));
                out.push_back(std::move(value));
            }
        }       
        return out;
    }

}