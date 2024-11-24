#include <testing_framework/core/cli.h>
#include <testing_framework/helpers/options.h>

int main(int argc, char**argv)
{
    
    testing::add_options();

    return !testing::handle_args(argc, argv);
}