#include <testing_framework/core/cli.h>
#include <testing_framework/helpers/init.h>

int main(int argc, char**argv)
{
    
    testing::init_helpers();

    return !testing::handle_args(argc, argv);
}