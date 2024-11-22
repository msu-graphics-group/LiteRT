#include <testing_framework/core/supervisor.h>
#include <testing_framework/core/logging.h>
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <string.h>
#include <sys/prctl.h>

namespace testing
{

    Supervisor::Supervisor() :
        status_(0),
        pid_(0),
        out_(-1)
    {}

    Supervisor::Supervisor(Supervisor::pid_t pid, int out) :
        status_(0),
        pid_(pid),
        out_(out)
    {}

    Supervisor::Supervisor(Supervisor&&other) :
        status_(other.status_),
        pid_(other.pid_),
        out_(other.out_)
    {
        other.status_ = 0;
        other.pid_ = 0;
        other.out_ = -1;
    }

    Supervisor& Supervisor::operator=(Supervisor&&other)
    {
        Supervisor tmp(std::move(other));
        swap(tmp);
        return *this;
    }
    Supervisor::~Supervisor()
    {
        clear();
    }

    void Supervisor::swap(Supervisor&other)
    {
        std::swap(status_, other.status_);
        std::swap(pid_, other.pid_);
        std::swap(out_, other.out_);
    }

    bool Supervisor::exited() const
    {
        return WIFEXITED(status_);
    }
    int Supervisor::exit_status() const
    {
        return WEXITSTATUS(status_);
    }

    bool Supervisor::spawn_call(Supervisor::pid_t&pid, int&out, void(*func)(void*), void*ctx)
    {

        int fd[2];
        if (pipe(fd))
        {
            std::cerr << foreground(error_color) << "Error: " << default_color
                << "failed to create pipe: " << strerror(errno) << "." << std::endl;
            return false;
        }

        auto close_read = [&]()->int{
            if (close(fd[0]))
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed to close pipe read end: " << strerror(errno) << "." << std::endl;
                return 1;
            }
            return 0;
        };
        auto close_write = [&]()->int{
            if (close(fd[1]))
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed to close pipe write end: " << strerror(errno) << "." << std::endl;
                return 1;
            }
            return 0;
        };

        pid_t parent_pid = getpid();

        pid_t res = fork();
        if (res == -1)
        {
            std::cerr << foreground(error_color) << "Error: " << default_color
                 << "failed to fork: " << strerror(errno) << "." << std::endl;
            close_read();
            close_write();
            return false;
        }
        if (res != 0) // parent
        {
            if (close_write())
            {
                close_read();
                return false;
            }
            pid = res;
            out = fd[0];
            return true;
        }
        else // child
        {
            if (prctl(PR_SET_PDEATHSIG, SIGKILL)) // child is killed after parent dies
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed syscall prctl(PR_SET_PDEATHSIG, SIGKILL): " << strerror(errno) << "." << std::endl;
                close_read();
                close_write();
                exit(1);
            }

            /*
                In case of parent dying, if prctl has been executed before parent died then this process will be killed.
                Otherwise parent was changed to init or subreaper and prctl has no effect.
                Easiest way to check this is to compare pids.
                This method is not ideal because it has ABA problem.
                I don't know how to do it better.
            */
            if (parent_pid != getppid())
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "supervisor process has died." << std::endl;
                close_read();
                close_write();
                exit(1);
            }

            if (close_read())
            {
                close_write();
                exit(1);
            }

            if (dup2(fd[1], 1) == -1)
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed to dup pipe write end to stdout: " << strerror(errno) << "." << std::endl;
                close_write();
                exit(1);
            }
            if (close_write())
            {
                exit(1);
            }
            func(ctx);
            return true; // probably never here
        }
    }

    template<typename F>
    bool Supervisor::spawn_call(Supervisor::pid_t&pid, int&out, F&&f)
    {
        F*ctx = &f;
        auto func = [](void*ctx) {
            (*static_cast<F*>(ctx))();
        };
        return spawn_call(pid, out, func, static_cast<void*>(ctx));
    }

    std::optional<Supervisor> Supervisor::spawn(const std::vector<std::string>&args)
    {
        pid_t pid;
        int out;
        bool res = spawn_call(pid, out, [&](){
            
            std::vector<char*> ptrs(args.size() + 1);
            
            for (size_t i = 0; i < args.size(); i++)
            {
                ptrs[i] = const_cast<char*>(args[i].c_str()); // probably ok
            }
            ptrs.back() = nullptr;

            execvp(ptrs[0], ptrs.data());

            // failed to run
            std::cerr << foreground(error_color) << "Error: " << default_color
                << "failed to exec '";
            for (size_t i = 0; i < args.size(); i++)
            {
                std::cerr << args[i];
                if (i != args.size() - 1)
                {
                    std::cerr << " ";
                }
            }
            std::cerr << "': " << strerror(errno) << "." << std::endl;

            exit(1);
        });
        if (res)
        {
            return Supervisor(pid, out);
        }
        return std::nullopt;
    }

    void Supervisor::clear()
    {
        if (pid_)
        {
            // I don't know, how to handle error
            // Only way is to ignore them or exit
            if (close(out_))
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed to close subprocess output: " << strerror(errno) << "." << std::endl;
            }
            if (kill(pid_, SIGKILL))
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed to kill subprocess: " << strerror(errno) << "." << std::endl;
            }
            if (wait(&status_) == -1)
            {
                std::cerr << foreground(error_color) << "Error: " << default_color
                    << "failed to wait for subprocess: " << strerror(errno) << "." << std::endl;
            }
            pid_ = 0;
            out_ = -1;
        }
    }

    std::optional<int> Supervisor::get_char()
    {
        if (pid_ == 0)
        {
            return EOF;
        }
        
        while (true)
        {
            char x;
            int res = read(out_, &x, 1);
            if (res == -1)
            {
                if (errno == EINTR) // retry
                {
                    continue;
                }
                else // really an error
                {
                    std::cerr << foreground(error_color) << "Error: " << default_color
                        << "failed to read subprocess output: " << strerror(errno) << "." << std::endl;
                    clear();
                    return std::nullopt;
                }
            }
            else if(res == 0)
            {
                clear();
                return int(EOF);
            }
            else // 1
            {
                return int(x);
            }
        }
    }

}