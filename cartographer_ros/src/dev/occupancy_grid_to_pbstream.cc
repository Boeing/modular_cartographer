#include <absl/strings/str_split.h>
#include <cartographer_ros/assets_writer.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <png.h>

#include <istream>
#include <iterator>

#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/io/proto_stream.h>
#include <cartographer/mapping/map_builder.h>

#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer/mapping/2d/probability_grid.h>
#include <cartographer/mapping/2d/submap_2d.h>
#include <cartographer/mapping/internal/2d/pose_graph_2d.h>
#include <cartographer/mapping/internal/optimization/optimization_problem_2d.h>
#include <cartographer_ros/proto_sstream.h>

#include <cartographer/io/internal/mapping_state_serialization.h>

#include <tinyxml.h>

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(pbstream, "", "Output location for the protobuf file");
DEFINE_string(map_png, "", "PNG of occupancy grid");
DEFINE_string(world_sdf, "", "SDF of the world");
DEFINE_string(pole_prefix, "bollard", "SDF model prefix for poles");

DEFINE_double(map_resolution, 0.02, "Resolution of map");
DEFINE_double(pole_radius, 0.07, "Radius of poles");


std::vector<const TiXmlElement*> constIterate(const TiXmlElement* elm, const std::string& name)
{

    std::vector<const TiXmlElement*> elms;
    TiXmlElement* child = const_cast<TiXmlElement*>(elm->FirstChildElement(name));
    while (child)
    {
        elms.push_back(child);
        child = child->NextSiblingElement(name);
    }
    return elms;
}

std::vector<Eigen::Vector2f> poleLocations(const TiXmlElement* sdf, const std::string& pole_prefix)
{
    std::vector<Eigen::Vector2f> poles;

    const auto world = sdf->FirstChildElement("world");
    if (!world)
        return poles;

    const auto state = world->FirstChildElement("state");
    if (!state)
        return poles;

    for (const TiXmlElement* model_state : constIterate(state, "model"))
    {
        const auto name = model_state->Attribute("name");
        if (name)
        {
            const std::string s_name = std::string(name);
            if (s_name.substr(0, pole_prefix.size()) == pole_prefix)
            {
                const auto pose = model_state->FirstChildElement("pose");
                CHECK(pose != nullptr);
                CHECK(pose->GetText() != nullptr);
                const std::string value = std::string(pose->GetText());
                std::istringstream iss(value);
                const std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                                       std::istream_iterator<std::string>());
                CHECK(results.size() == 6);
                poles.push_back({std::stof(results.at(0)), std::stof(results.at(1))});
            }
        }
    }

    return poles;
}

int main(int argc, char** argv)
{
    FLAGS_alsologtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty()) << "-configuration_directory is missing.";
    CHECK(!FLAGS_pbstream.empty()) << "-pose_graph_filename is missing.";
    CHECK(!FLAGS_map_png.empty()) << "-map_png is missing.";
    CHECK(!FLAGS_world_sdf.empty()) << "-world_sdf is missing.";

    unsigned int width;
    unsigned int height;
    png_bytep* row_pointers = nullptr;
    {
        FILE* fp = fopen(FLAGS_map_png.c_str(), "rb");

        png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
        if (!png)
            abort();

        png_infop info = png_create_info_struct(png);
        if (!info)
            abort();

        if (setjmp(png_jmpbuf(png)))
            abort();

        png_init_io(png, fp);
        png_read_info(png, info);

        width = png_get_image_width(png, info);
        height = png_get_image_height(png, info);
        const png_byte color_type = png_get_color_type(png, info);
        const png_byte bit_depth = png_get_bit_depth(png, info);

        if (bit_depth != 8)
            throw std::runtime_error("PNG must be 8 bit");

        if (color_type != PNG_COLOR_TYPE_GRAY)
            throw std::runtime_error("PNG must be grayscale");

        png_read_update_info(png, info);

        row_pointers = static_cast<png_bytep*>(malloc(sizeof(png_bytep) * height));
        for (unsigned int y = 0; y < height; y++)
        {
            row_pointers[y] = static_cast<png_byte*>(malloc(png_get_rowbytes(png, info)));
        }
        png_read_image(png, row_pointers);

        fclose(fp);

        png_destroy_read_struct(&png, &info, nullptr);
    }

    auto file_resolver = absl::make_unique<cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{FLAGS_configuration_directory});
    const std::string code = file_resolver->GetFileContentOrDie("cartographer.lua");
    auto lua_parameter_dictionary =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(code, std::move(file_resolver));
    const auto map_builder_options =
        cartographer::mapping::CreateMapBuilderOptions(lua_parameter_dictionary->GetDictionary("map_builder").get());
    const auto trajectory_builder_options = cartographer::mapping::CreateTrajectoryBuilderOptions(
        lua_parameter_dictionary->GetDictionary("trajectory_builder").get());

    LOG(INFO) << "width_px: " << width;
    LOG(INFO) << "height_px: " << height;

    const double width_m = width * FLAGS_map_resolution;
    const double height_m = height * FLAGS_map_resolution;

    LOG(INFO) << "width_m: " << width_m;
    LOG(INFO) << "height_m: " << height_m;

    cartographer::mapping::ValueConversionTables conversion_tables;
    auto grid = absl::make_unique<cartographer::mapping::ProbabilityGrid>(
        cartographer::mapping::MapLimits(
            FLAGS_map_resolution, Eigen::Vector2d(width_m, height_m),
            cartographer::mapping::CellLimits(static_cast<int>(height), static_cast<int>(width))),
        &conversion_tables);

    for (unsigned int y = 0; y < height; y++)
    {
        png_bytep row = row_pointers[y];
        for (unsigned int x = 0; x < width; x++)
        {
            const png_byte px = row[x];

            const unsigned int mirrored_x = width - x - 1;
            const unsigned int mirrored_y = height - y - 1;

            if (px == 100)
            {
                grid->SetProbability({mirrored_y, mirrored_x}, 1.f);
            }
            else if (px < 100)
            {
                grid->SetProbability({mirrored_y, mirrored_x}, static_cast<float>(px) / 100.f);
            }
        }
    }
    grid->FinishUpdate();

    for (unsigned int y = 0; y < height; y++)
    {
        free(row_pointers[y]);
    }
    free(row_pointers);

    const Eigen::Vector2f origin = {width_m / 2.0, height_m / 2.0};
    cartographer::mapping::Submap2D submap(origin, std::move(grid), &conversion_tables);

    if (!FLAGS_world_sdf.empty())
    {
        LOG(INFO) << "Searching for poles in " << FLAGS_world_sdf << " with prefix " << FLAGS_pole_prefix;

        TiXmlDocument doc;
        doc.LoadFile(FLAGS_world_sdf.c_str());
        TiXmlHandle handle(&doc);
        TiXmlElement* root = handle.FirstChildElement("sdf").ToElement();
        CHECK(root != nullptr);
        const auto poles = poleLocations(root, FLAGS_pole_prefix);
        std::vector<cartographer::mapping::CircleFeature> cf;
        for (const Eigen::Vector2f& p : poles)
        {
            LOG(INFO) << "Found pole: " << p.transpose();
            cartographer::mapping::CircleFeature f;
            f.keypoint.position.x() = p.x();
            f.keypoint.position.y() = p.y();
            f.keypoint.position.z() = 0.0;
            f.fdescriptor.score = 0.0;
            f.fdescriptor.radius = static_cast<float>(FLAGS_pole_radius);
            cf.push_back(f);
        }
        submap.SetCircleFeatures(cf);
    }

    submap.Finish();

    cartographer::common::ThreadPool thread_pool(1);

    auto opt_problem = absl::make_unique<cartographer::mapping::optimization::OptimizationProblem2D>(
        map_builder_options.pose_graph_options().optimization_problem_options());

    cartographer::mapping::PoseGraph2D pose_graph(map_builder_options.pose_graph_options(), std::move(opt_problem),
                                                  &thread_pool);

    cartographer::mapping::proto::Submap submap_proto = submap.ToProto(true);
    submap_proto.mutable_submap_id()->set_trajectory_id(0);
    submap_proto.mutable_submap_id()->set_submap_index(0);

    cartographer::transform::Rigid3d global_submap_pose(cartographer::transform::Rigid3d::Vector(0, 0, 0),
                                                        cartographer::transform::Rigid3d::Quaternion(1, 0, 0, 0));
    pose_graph.AddSubmapFromProto(global_submap_pose, submap_proto);
    pose_graph.FinishTrajectory(0);
    pose_graph.RunFinalOptimization();
    pose_graph.FreezeTrajectory(0);

    cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds t_opt;
    *t_opt.mutable_trajectory_builder_options() = trajectory_builder_options;

    std::vector<cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds> t_opts;
    t_opts.push_back(t_opt);

    LOG(INFO) << "Writing pbstream to: " << FLAGS_pbstream;
    cartographer::io::ProtoStreamWriter writer(FLAGS_pbstream);
    cartographer::io::WritePbStream(pose_graph, t_opts, &writer, true);
    writer.Close();
}
