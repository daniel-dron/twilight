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
#include <vulkan/vulkan_core.h>
#include "r_shaders.h"

using namespace tl;

static VkPipelineColorBlendAttachmentState create_blend_attachment_state( const PipelineConfig::BlendType& blend_type ) {
    if ( blend_type == PipelineConfig::BlendType::ADDITIVE ) {
        return {
                .blendEnable         = VK_TRUE,
                .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
                .dstColorBlendFactor = VK_BLEND_FACTOR_ONE,
                .colorBlendOp        = VK_BLEND_OP_ADD,
                .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
                .dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO,
                .alphaBlendOp        = VK_BLEND_OP_ADD,
                .colorWriteMask      = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT };
    }

    if ( blend_type == PipelineConfig::BlendType::ALPHA_BLEND ) {
        return {
                .blendEnable         = VK_TRUE,
                .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
                .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
                .colorBlendOp        = VK_BLEND_OP_ADD,
                .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
                .dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO,
                .alphaBlendOp        = VK_BLEND_OP_ADD,
                .colorWriteMask      = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT };
    }

    if ( blend_type == PipelineConfig::BlendType::OFF ) {
        return {
                .blendEnable    = VK_FALSE,
                .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT };
    }

    std::unreachable( );
}

void Pipeline::initialize( const PipelineConfig& config ) {
    if ( config.compute != nullptr ) {
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
    else {
        VkShaderModule vertex, mesh, task;

        std::vector<VkPipelineShaderStageCreateInfo> shader_stages_info;

        if ( config.vertex ) {
            vertex = load_shader_module( config.vertex );
            assert( vertex != VK_NULL_HANDLE && "Could not find vertex shader" );
            shader_stages_info.emplace_back( VkPipelineShaderStageCreateInfo{ .sType               = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                                                                              .pNext               = nullptr,
                                                                              .flags               = 0,
                                                                              .stage               = VK_SHADER_STAGE_VERTEX_BIT,
                                                                              .module              = vertex,
                                                                              .pName               = "main",
                                                                              .pSpecializationInfo = nullptr } );
        }
        else if ( config.mesh ) {
            if ( config.task ) {
                task = load_shader_module( config.task );
                assert( task != VK_NULL_HANDLE && "Could not find task shader" );
                shader_stages_info.emplace_back( VkPipelineShaderStageCreateInfo{ .sType               = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                                                                                  .pNext               = nullptr,
                                                                                  .flags               = 0,
                                                                                  .stage               = VK_SHADER_STAGE_TASK_BIT_EXT,
                                                                                  .module              = task,
                                                                                  .pName               = "main",
                                                                                  .pSpecializationInfo = nullptr } );
            }

            mesh = load_shader_module( config.mesh );
            assert( mesh != VK_NULL_HANDLE && "Could not find mesh shader" );
            shader_stages_info.emplace_back( VkPipelineShaderStageCreateInfo{ .sType               = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                                                                              .pNext               = nullptr,
                                                                              .flags               = 0,
                                                                              .stage               = VK_SHADER_STAGE_MESH_BIT_EXT,
                                                                              .module              = mesh,
                                                                              .pName               = "main",
                                                                              .pSpecializationInfo = nullptr } );
        }

        const auto pixel = load_shader_module( config.pixel );
        assert( pixel != VK_NULL_HANDLE && "Could not find pixel shader" );
        shader_stages_info.emplace_back( VkPipelineShaderStageCreateInfo{ .sType               = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                                                                          .pNext               = nullptr,
                                                                          .flags               = 0,
                                                                          .stage               = VK_SHADER_STAGE_FRAGMENT_BIT,
                                                                          .module              = pixel,
                                                                          .pName               = "main",
                                                                          .pSpecializationInfo = nullptr } );

        std::vector<VkFormat>                            color_formats;
        std::vector<VkPipelineColorBlendAttachmentState> blend_attachments;
        color_formats.reserve( config.color_targets.size( ) );
        blend_attachments.reserve( config.color_targets.size( ) );
        for ( auto& color_targets_config : config.color_targets ) {
            color_formats.push_back( color_targets_config.format );
            blend_attachments.push_back( create_blend_attachment_state( color_targets_config.blend_type ) );
        }

        // We do not make use of render passes, thus we need to create a Rendering Info structure
        VkPipelineRenderingCreateInfo render_info = { .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
                                                      .pNext                   = nullptr,
                                                      .colorAttachmentCount    = static_cast<uint32_t>( color_formats.size( ) ),
                                                      .pColorAttachmentFormats = color_formats.data( ),
                                                      .depthAttachmentFormat   = VK_FORMAT_D32_SFLOAT };

        // No need to configribe viewports and scissor state here since we will be using dynamic states
        VkPipelineViewportStateCreateInfo viewport_info = {
                .sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
                .pNext         = nullptr,
                .viewportCount = 1,
                .scissorCount  = 1 };


        VkPipelineColorBlendStateCreateInfo blend_info = {
                .sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
                .pNext           = nullptr,
                .logicOpEnable   = VK_FALSE,
                .attachmentCount = static_cast<uint32_t>( blend_attachments.size( ) ),
                .pAttachments    = blend_attachments.size( ) == 0 ? nullptr : blend_attachments.data( ) };

        VkPipelineVertexInputStateCreateInfo vertex_info = { .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };

        // We use dynamic state for scissor and viewport. This, together with dynamic rendering, makes it really
        // nice and simple to use modern vulkan
        std::array<VkDynamicState, 2>    state        = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamic_info = {
                .sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
                .pNext             = nullptr,
                .dynamicStateCount = static_cast<uint32_t>( state.size( ) ),
                .pDynamicStates    = state.data( ) };

        VkPipelineInputAssemblyStateCreateInfo input_info = {
                .sType    = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
                .pNext    = nullptr,
                .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST };

        VkPipelineRasterizationStateCreateInfo raster_info = {
                .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
                .pNext = nullptr,

                .polygonMode = config.polygon_mode,
                .cullMode    = config.cull_mode,
                .frontFace   = config.front_face,
                .lineWidth   = config.line_width };

        // TODO: expose this through the parameter structure
        VkPipelineMultisampleStateCreateInfo multisample_info = {
                .sType                 = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
                .pNext                 = nullptr,
                .rasterizationSamples  = VK_SAMPLE_COUNT_1_BIT,
                .sampleShadingEnable   = VK_FALSE,
                .minSampleShading      = 1.0f,
                .pSampleMask           = nullptr,
                .alphaToCoverageEnable = VK_FALSE,
                .alphaToOneEnable      = VK_FALSE };

        VkPipelineDepthStencilStateCreateInfo depth_stencil_info = {
                .sType             = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
                .pNext             = nullptr,
                .depthTestEnable   = config.depth_test,
                .depthWriteEnable  = config.depth_write,
                .depthCompareOp    = config.depth_compare,
                .stencilTestEnable = false,
                .front             = { },
                .back              = { },
                .minDepthBounds    = 1.0f,
                .maxDepthBounds    = 0.0f };

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

        VkGraphicsPipelineCreateInfo pipeline_info = { .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
                                                       .pNext = &render_info,

                                                       .stageCount =
                                                               static_cast<uint32_t>( shader_stages_info.size( ) ),
                                                       .pStages             = shader_stages_info.data( ),
                                                       .pVertexInputState   = &vertex_info,
                                                       .pInputAssemblyState = &input_info,
                                                       .pViewportState      = &viewport_info,
                                                       .pRasterizationState = &raster_info,
                                                       .pMultisampleState   = &multisample_info,
                                                       .pDepthStencilState  = &depth_stencil_info,
                                                       .pColorBlendState    = &blend_info,
                                                       .pDynamicState       = &dynamic_info,
                                                       .layout              = layout };

        if ( vkCreateGraphicsPipelines( g_ctx.device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline ) !=
             VK_SUCCESS ) {
            assert( false && "Failed to create pipeline" );
        }

        // We no longer need the shader modules
        if ( config.vertex ) {
            vkDestroyShaderModule( g_ctx.device, vertex, nullptr );
        }
        else {
            vkDestroyShaderModule( g_ctx.device, mesh, nullptr );

            if ( config.task ) {
                vkDestroyShaderModule( g_ctx.device, task, nullptr );
            }
        }
        vkDestroyShaderModule( g_ctx.device, pixel, nullptr );
    }
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