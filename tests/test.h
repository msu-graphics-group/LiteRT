#pragma once
#include <string>
#include <vector>

namespace test
{

    struct test_execution_context
    {
        bool rewrite;
    };

    enum class test_result
    {
        passed,
        skipped,
        failed,
        crashed
    };

    enum class check_result
    {
        passed,
        failed
    };

    struct test_execution_info
    {
        size_t passed_checks;
        size_t failed_checks;
        bool was_skipped;
        bool was_crashed;
    };

    /*
        Returns true if test is in rewrite mode
    */
    const test_execution_context&test_context();

    /*
        Adds check result to current test info
    */
    void add_check_result(check_result);

    /*
        Skips current test
    */
    void skip();

    class Test
    {
    public:
        
        Test(std::string run_name);

        std::string_view run_name() const;

        static const std::vector<const Test*> all();
        
        static void list();
        static bool run(const std::vector<const Test*>&tests);
        
        void unsafe_run(test_execution_context) const;

    private:
        
        test_execution_info unsafe_execute(test_execution_context) const;
        bool supervised_execute(test_execution_info&info) const;

        void supervised_child() const;
        bool supervised_parent(int child_pid, int child_output, test_execution_info&info) const;

        /*
            Method that contains test code
            Needs to be overwritten in every test
        */
        virtual void test_code() const = 0;

        std::string run_name_;
        static std::vector<const Test*>& tests();
    };

}

#define __ADD_TEST(test_id, run_name)     \
    class test_id : public ::test::Test { \
        using ::test::Test::Test;         \
        void test_code() const override;        \
    };                                    \
    test_id test_id##_instance(run_name);           \
    void test_id::test_code() const 

#define ADD_TEST(name, run_name)          \
    __ADD_TEST(Test_##name, run_name)
