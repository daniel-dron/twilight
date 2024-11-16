/******************************************************************************
******************************************************************************
**                                                                           **
**                             Twilight Engine                               **
**                                                                           **
**                  Copyright (c) 2024-present Daniel Dron                   **
**                                                                           **
**            This software is released under the MIT License.               **
**                 https://opensource.org/licenses/MIT                       **
**                                                                           **
******************************************************************************
******************************************************************************/
#include <pch.h>

#include <Volk/volk.h>
#include <print>
#include <vulkan/vulkan_core.h>
#include "VkBootstrap.h"

#include "r_swapchain.h"

using namespace tl;

void Swapchain::initialize( u32 width, u32 height, VkPhysicalDevice physical_device, VkDevice device, VkSurfaceKHR surface ) {
    create( width, height, physical_device, device, surface, VK_NULL_HANDLE );
}

void Swapchain::shutdown( VkDevice device ) {
    vkDestroySwapchainKHR( device, swapchain, nullptr );
}

void Swapchain::resize( u32 width, u32 height, VkPhysicalDevice physical_device, VkDevice device, VkSurfaceKHR surface ) {
    create( width, height, physical_device, device, surface, this->swapchain );
}

void Swapchain::create( u32 width, u32 height, VkPhysicalDevice physical_device, VkDevice device, VkSurfaceKHR surface, VkSwapchainKHR old_swapchain ) {
    this->width  = width;
    this->height = height;

    std::println( "Creating swapchain {}x{}", width, height );

    vkb::SwapchainBuilder builder{ physical_device, device, surface };

    auto swapchain = builder.use_default_format_selection( )
                             .set_old_swapchain( old_swapchain )
                             .set_desired_present_mode( VK_PRESENT_MODE_FIFO_KHR )
                             .add_image_usage_flags( VK_IMAGE_USAGE_TRANSFER_DST_BIT ) // to allow copies to it
                             .build( );
    assert( swapchain.has_value( ) );
    this->swapchain = swapchain.value( );

    vkDestroySwapchainKHR( device, old_swapchain, nullptr );

    images = swapchain->get_images( ).value( );
    assert( images.size( ) != 0 );
}