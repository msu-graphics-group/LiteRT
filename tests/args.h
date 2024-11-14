#pragma once
#include <string>
#include <vector>
#include <map>
#include <optional>

namespace test
{

    /*
        Parses arguments in form
        --name value1 value2 ...

    */
    class Args
    {
    public:
        
        static bool is_arg_name(std::string_view);
        /*
            If argument list have appopriate format then parses it and return
            Only incorrect case is start from non-name (no --smth)
        */
        static std::optional<Args> parse(size_t argc, char**argv);

        /*
            Checks if there aren't arguments with unspecified names
        */
        bool check_only(const std::vector<std::string_view>&names) const;

        /*
            If there is no argument with specified name, then returns true
            If there is argument with specified name and it has specified type, then returns it value
            If arguments exists but type is incorrec returns false
        */
        bool get(std::string_view name, bool&flag) const;
        bool get(std::string_view name, std::optional<std::string_view>&value) const;
        bool get(std::string_view name, std::optional<std::vector<std::string_view>>&list) const;
        
    private:
        std::map<std::string_view, std::vector<std::string_view>> args_;
    };

}