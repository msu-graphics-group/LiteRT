#pragma once
#include <optional>
#include <string>
#include <vector>

namespace testing
{

    /*
        Class for managing spawned processes
    */
    class Supervisor
    {
    public:

        Supervisor();
        Supervisor(const Supervisor&other) = delete;
        Supervisor(Supervisor&&other);
        Supervisor& operator=(const Supervisor&other) = delete;
        Supervisor& operator=(Supervisor&&other);
        ~Supervisor();

        void swap(Supervisor&other);
        void clear();

        /*
            Blocking read from subprocess stdout
            Returns EOF if nothing to read (subprocess finished)
            Returns std::nullopt if error happend, subprocess is killed
        */
        std::optional<int> get_char();

        bool exited() const;
        int exit_status() const;

        /*
            Creates a subprocess with stdout redirected to pipe
        */
        static std::optional<Supervisor> spawn(const std::vector<std::string>&args);

    private:

        using pid_t = int;

        /*
            Creates subprocess and calls func(ctx) in it
            Returns pid and file descriptor to it's output file
        */
        static bool spawn_call(pid_t&pid, int&out, void(*func)(void*ctx), void*ctx);

        /*
            Creates subprocess and calls func() in it
            Returns pid and file descriptor to it's output file
        */
        template<typename F>
        static bool spawn_call(pid_t&pid, int&out, F&&func);

        Supervisor(pid_t pid, int out);

        int status_;
        int pid_;
        int out_;
    };

}