#include <testing_framework/helpers/files.h>
#include <testing_framework/core/logging.h>
#include <testing_framework/core/environment.h>
#include <filesystem>

namespace testing
{

    bool file_exists(const std::string &path)
    {
        return std::filesystem::exists(path);
    }

    /*
        If there is no file, skips test
    */
    bool assert_file_existance(const std::string &path, bool failure_as_error)
    {
        if (!file_exists(path))
        {
            log(failure_as_error ? bar_error : bar_warning) << "File "
                                                            << foreground(highlight_color_2) << path << default_color
                                                            << " does not exist" << std::endl;
            if (failure_as_error)
            {
                skip();
            }
            else
            {
                return false;
            }
        }
        return true;
    }

    /*
        Creates direcoty for path
    */
    void prepare_directoty_for_saving(const std::string &path)
    {
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    }

}