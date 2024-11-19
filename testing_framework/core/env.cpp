#include "env.h"
#include <iostream>
#include <algorithm>

namespace testing
{   

    /*
        Defined in actions.cpp
    */
    bool rewrite_is_enabled();

    bool ask_rewrite(std::string_view comment)
    {

        static std::string yes[] = {"yes", "Y", "y"};
        static std::string no[] = {"no", "N", "n"};

        std::cout <<  "Are you sure you want to " << comment << "?" << std::endl;
        
        while (true)
        {
            std::cout << "Please, answer [y/n]";
            std::string line;
            std::getline(std::cin, line);
            if (std::cin.eof())
            {
                // clear exit
                std::cout << std::endl;
                skip();
            }
            if (line.size() > 0 && line.back() == '\n')
            {
                line.resize(line.size() - 1);
            }
            auto y = std::find(std::begin(yes), std::end(yes), line);
            auto n = std::find(std::begin(no), std::end(no), line);

            if (y != std::end(yes))
            {
                return true;
            }
            else if(n != std::end(no))
            {
                return false;
            }

        }


    }

    bool should_rewrite_saved_reference(std::string_view comment)
    {
        return rewrite_is_enabled() && ask_rewrite(comment);
    }

}