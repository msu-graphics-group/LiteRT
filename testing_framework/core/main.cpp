#include <testing_framework/core/cli.h>
#include <testing_framework/core/test_options.h>

int main(int argc, char**argv)
{
    return !testing::handle_args(argc, argv);
}