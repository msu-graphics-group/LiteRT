#include <testing_framework/helpers/init.h>
#include <testing_framework/helpers/options.h>
#include <testing_framework/core/filters.h>

namespace testing
{

    void init_helpers()
    {
        add_options();

        /*
            Filter annoying Vulkan messages
        */
        add_filter_if_contains("WARNING-CreateInstance-status-message");

        add_filter_if_contains("Current Enables: VK_VALIDATION_FEATURE_ENABLE_DEBUG_PRINTF_EXT.");
        add_filter_if_contains("Current Disables: None.");
        add_filter_if_contains("Objects: 1");
        add_filter_if_contains("type: 1, name: NULL");

        add_filter_if_contains("WARNING-cache-file-error");
        
        add_filter_if_contains("VkUtils::INFO");
    
    }

}