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

#pragma once

#include <Volk/volk.h>
#include <unordered_map>
#include "r_resources.h"

namespace tl {
    struct PipelineConfig {
        const char* const name    = nullptr;
        const char* const vertex  = nullptr;
        const char* const pixel   = nullptr;
        const char* const compute = nullptr;
        const char* const mesh    = nullptr;
        const char* const task    = nullptr;

        VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
        float         line_width   = 1.0f;

        VkCullModeFlags cull_mode  = VK_CULL_MODE_BACK_BIT;
        VkFrontFace     front_face = VK_FRONT_FACE_CLOCKWISE;

        bool        depth_test    = true;
        bool        depth_write   = true;
        VkCompareOp depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;

        enum class BlendType { OFF,
                               ADDITIVE,
                               ALPHA_BLEND,
                               ACCUMULATE };
        struct ColorTargetsConfig {
            VkFormat  format     = VK_FORMAT_R8G8B8A8_SRGB;
            BlendType blend_type = BlendType::OFF;
        };
        std::vector<ColorTargetsConfig> color_targets;

        // TODO: In the future, would be nice to extrapolate the layout from the vertex & pixel shaders directly
        // For now, this will do...
        std::vector<VkPushConstantRange>   push_constant_ranges;
        std::vector<VkDescriptorSetLayout> descriptor_set_layouts;
    };

    enum class BindingsStage : u32 {
        COMPUTE,
        TASK,
        MESH,
        VERTEX,
        FRAGMENT
    };

    struct Pipeline {
        VkPipeline       pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout   = VK_NULL_HANDLE;

        std::unordered_map<BindingsStage, ShaderBindings> bindings = { };

        void initialize( const PipelineConfig& config );
        void shutdown( );
    };

    VkShaderModule load_shader_module( const char* path );

} // namespace tl