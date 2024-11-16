#pragma once
#include <string>

namespace test
{

    class Supervisor
    {
    public:

        Supervisor(const char**prog, int argc, char**argv);

        /*
            if no error happens retuens true and line
            Otherwise returns false

            Empty line means end
        */
        bool get_line(std::string&line);
        bool exited() const;
        int return_code() const;

    private:
        int status_;
        int out;
        int pid;
    };

}