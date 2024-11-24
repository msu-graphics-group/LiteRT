#include <testing_framework/helpers/mesh.h>
#include <testing_framework/helpers/options.h>
#include <testing_framework/helpers/files.h>
#include <testing_framework/helpers/format.h>
#include <testing_framework/core/logging.h>

namespace testing
{

   
    Mesh load_vsgf_mesh_by_path(const std::string &path)
    {
        log(bar_info) << "Loading VSGF mesh from "
                      << foreground(highlight_color_2) << path << default_color
                      << std::endl;
        assert_file_existance(path);
        auto mesh = cmesh4::LoadMeshFromVSGF(path.c_str());
        log(bar_info) << "Mesh triangles count: "
                      << foreground(highlight_color_3) << mesh.TrianglesNum()
                      << std::endl;
        return mesh;
    }

    void fit_mesh(Mesh &mesh, LiteMath::float3 box_min, LiteMath::float3 box_max)
    {
        LiteMath::float3 min, max;
        get_bbox(mesh, &min, &max);
        
        log(bar_info) << std::fixed << std::setprecision(3) << "Mesh bounding box: "
                      << "["
                      << foreground(highlight_color_1) << min << default_color
                      << " - "
                      << foreground(highlight_color_1) << max << default_color
                      << "]" << std::endl;
        log(bar_info) << "Transforming mesh to fit into bounding box "
                      << "["
                      << foreground(highlight_color_1) << box_min << default_color
                      << " - "
                      << foreground(highlight_color_1) << box_max << default_color
                      << "]" << std::endl;
        rescale_mesh(mesh, box_min, box_max);
        get_bbox(mesh, &min, &max);
        log(bar_info) << "Mesh bounding box after transformation: "
                      << "["
                      << foreground(highlight_color_1) << min << default_color
                      << " - "
                      << foreground(highlight_color_1) << max << default_color
                      << "]" << std::endl;
    }

    Mesh load_vsgf_mesh(const std::string &name, float box)
    {
        std::string path = scenes_directory() + "/" + name;
        auto mesh = load_vsgf_mesh_by_path(path);
        LiteMath::float3 v(1, 1, 1);
        fit_mesh(mesh, -v * box, v * box);
        return mesh;
    }

}