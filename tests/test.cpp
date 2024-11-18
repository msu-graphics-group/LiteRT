#include "test.h"

namespace test
{

    // std::vector<const Test*> Test::tests_;

    Test::Test(std::string run_name) :
        run_name_(std::move(run_name))
    {
        tests().push_back(this);
    }

    std::vector<const Test*>& Test::tests()
    {
        static std::vector<const Test*> tests_;
        return tests_;
    }

    std::string_view Test::run_name() const 
    {
        return run_name_;
    }

    const std::vector<const Test*> Test::all()
    {
        return tests();
    }

    void Test::list()
    {

    }

    bool Test::run(const std::vector<const Test*>&tests)
    {
        for (auto i : tests)
        {
            test_execution_info info;
            bool res = i->supervised_execute(info);
            if (!res)
            {
                return false;
            }
        }
        return true;
    }
    
    void Test::unsafe_run(test_execution_context ctx) const
    {
        auto res = unsafe_execute(ctx);

        std::cout << res.passed_checks << " " << res.failed_checks<< " " << res.was_skipped << std::endl;
        
    }

    bool Test::supervised_execute(test_execution_info&info) const
    {
        int p[2];
        if (pipe(p))
        {
            std::cerr << "Failed to pipe: " << strerror(errno) << std::endl;
            return false;
        }

        pid_t pid = fork();
        if (pid == -1)
        {
            std::cerr << "Failed to fork: " << strerror(errno) << std::endl;
            return false;
        }
        if (pid) // parent
        {
            close(p[1]);
            return supervised_parent(pid, p[0], info);
        } else // child
        {
            prctl(PR_SET_PDEATHSIG, SIGKILL); // child is killed after parent dies
            close(p[0]);
            dup2(p[1], 1);
            close(p[1]);
            supervised_child();
            return false; // never be here
        }

    }

    bool Test::supervised_parent(int child, int out, test_execution_info&info) const
    {
        std::string prev_line;
        std::string curr_line;
        while (true)
        {
            char x;
            int res = read(out, &x, 1);
            if (res == -1)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                else
                {
                    kill(child, SIGKILL);
                    wait(NULL);
                    std::cerr << "Failed to read: " << strerror(errno) << std::endl;
                    return false;
                }
            }
            else if(res == 1)
            {
                std::cout << x;
                curr_line.push_back(x);
                if (x == '\n')
                {
                    prev_line = curr_line;
                    curr_line = "";
                }
            }
            else
            {
                break;
            }
        }
        close(out);
        std::string last_line = curr_line == "" ? prev_line : curr_line;
        std::cout << "Child last line: '" << last_line << "'" << std::endl;
        
        return true;
    }

    void Test::supervised_child() const
    {
        std::cout << "Some text" << std::endl;
        std::cout << "123" << std::endl;
        std::cout << 4546 << std::endl;
        sleep(2);
        exit(0);
    }

    test_execution_info Test::unsafe_execute(test_execution_context) const
    {

    }

}