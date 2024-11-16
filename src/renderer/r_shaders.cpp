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

#include <renderer/r_context.h>
#include <types.h>
#include "r_shaders.h"

using namespace tl;

void Pipeline::initialize( const PipelineConfig& config ) {
    // TODO: implement graphics pipeline
    assert( config.pixel == nullptr );
    assert( config.vertex == nullptr );
    assert( config.compute != nullptr );

    auto compute = load_shader_module( config.compute );
    assert( compute );

    VkPipelineLayoutCreateInfo layout_info = {
            .sType          = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
            .pNext          = nullptr,
            .flags          = 0,
            .setLayoutCount = static_cast<uint32_t>( config.descriptor_set_layouts.size( ) ),
            .pSetLayouts    = config.descriptor_set_layouts.empty( ) ? nullptr : config.descriptor_set_layouts.data( ),

            .pushConstantRangeCount = static_cast<uint32_t>( config.push_constant_ranges.size( ) ),
            .pPushConstantRanges =
                    config.push_constant_ranges.empty( ) ? nullptr : config.push_constant_ranges.data( ) };

    VKCALL( vkCreatePipelineLayout( g_ctx.device, &layout_info, nullptr, &layout ) );

    const VkPipelineShaderStageCreateInfo stage_create_info = {
            .sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext  = nullptr,
            .stage  = VK_SHADER_STAGE_COMPUTE_BIT,
            .module = compute,
            .pName  = "main",
    };

    VkComputePipelineCreateInfo create_info{ };
    create_info.sType  = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    create_info.pNext  = nullptr;
    create_info.layout = layout;
    create_info.stage  = stage_create_info;

    VKCALL( vkCreateComputePipelines( g_ctx.device, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline ) );

    vkDestroyShaderModule( g_ctx.device, compute, nullptr );
}

void Pipeline::shutdown( ) {
    vkDestroyPipelineLayout( g_ctx.device, layout, nullptr );
    vkDestroyPipeline( g_ctx.device, pipeline, nullptr );
}

VkShaderModule tl::load_shader_module( const char* path ) {
    std::ifstream file( path, std::ios::ate | std::ios::binary );
    if ( !file.is_open( ) ) {
        return VK_NULL_HANDLE;
    }

    const size_t          file_size = file.tellg( );
    std::vector<uint32_t> buffer( file_size / sizeof( uint32_t ) );

    file.seekg( 0 );
    file.read( reinterpret_cast<char*>( buffer.data( ) ), file_size );
    file.close( );

    const VkShaderModuleCreateInfo create_info = {
            .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
            .pNext = nullptr,

            .codeSize = buffer.size( ) * sizeof( uint32_t ),
            .pCode    = buffer.data( ),
    };

    VkShaderModule shader_module;
    if ( vkCreateShaderModule( g_ctx.device, &create_info, nullptr, &shader_module ) != VK_SUCCESS ) {
        return VK_NULL_HANDLE;
    }
    return shader_module;
}