#include <absl/strings/str_split.h>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/io/internal/mapping_state_serialization.h>
#include <cartographer/io/proto_stream.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer/mapping/2d/probability_grid.h>
#include <cartographer/mapping/2d/submap_2d.h>
#include <cartographer/mapping/internal/2d/pose_graph_2d.h>
#include <cartographer/mapping/internal/optimization/optimization_problem_2d.h>
#include <cartographer/mapping/map_builder.h>
#include <cartographer_ros/assets_writer.h>
#include <cartographer_ros/proto_sstream.h>
#include <glog/logging.h>
#include <png.h>
#include <tinyxml.h>

#include <istream>
#include <iterator>

namespace po = boost::program_options;

std::string safe_string(const char* str)
{
    if (str)
        return std::string(str);
    return std::string();
}

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

Eigen::Isometry3f getPose(const TiXmlElement* pose)
{
    if (!pose || !pose->GetText())
        return Eigen::Isometry3f::Identity();

    std::string value = std::string(pose->GetText());

    std::istringstream iss(value);
    std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());

    const float x = std::stof(results.at(0));
    const float y = std::stof(results.at(1));
    const float z = std::stof(results.at(2));

    // Need to convert from rpy extrinsic to intrinsic
    // ((roll, pitch, yaw) - XYZ intrinsic) == ((yaw, pitch, roll) - ZYX extrinsic)
    const float roll = std::stof(results.at(5));
    const float pitch = std::stof(results.at(4));
    const float yaw = std::stof(results.at(3));

    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitX());

    return Eigen::Translation3f(x, y, z) * q;
}

std::vector<Eigen::Vector2d> getPolyline(const TiXmlElement* polyline)
{
    std::vector<Eigen::Vector2d> data;
    for (const TiXmlElement* point : constIterate(polyline, "point"))
    {
        CHECK(point);
        CHECK(point->GetText());
        const std::string value = std::string(point->GetText());
        std::istringstream iss(value);
        std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
        CHECK(results.size() == 2);
        const double x = std::stod(results.at(0));
        const double y = std::stod(results.at(1));
        data.push_back(Eigen::Vector2d{x, y});
    }
    return data;
}

Eigen::Vector2d transformPoint(const Eigen::Vector2d& point, const Eigen::Isometry3d& transform)
{
    return (transform * Eigen::Vector3d{point.x(), point.y(), 0.0}).head<2>();
}

std::vector<Eigen::Vector2d> transformPoints(const std::vector<Eigen::Vector2d>& points,
                                             const Eigen::Isometry3d& transform)
{
    std::vector<Eigen::Vector2d> transformed(points.size());
    std::transform(points.begin(), points.end(), transformed.begin(),
                   [&transform](const Eigen::Vector2d& point) { return transformPoint(point, transform); });
    return transformed;
}

Eigen::Vector2i worldToMap(const Eigen::Vector2d& point, const Eigen::Vector2d& map_origin, const double map_resolution)
{
    return Eigen::Vector2i{static_cast<int>((point.x() - map_origin.x()) / map_resolution + 0.5),
                           static_cast<int>((point.y() - map_origin.y()) / map_resolution + 0.5)};
}

std::vector<Eigen::Vector2i> worldToMap(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& map_origin,
                                        const double map_resolution)
{
    std::vector<Eigen::Vector2i> cells(points.size());
    std::transform(points.begin(), points.end(), cells.begin(),
                   [&map_origin, &map_resolution](const Eigen::Vector2d& point) {
                       return worldToMap(point, map_origin, map_resolution);
                   });
    return cells;
}

void drawPolyline(cairo_t* cr, const std::vector<Eigen::Vector2i>& polyline, const double border_width,
                  const double fill_color, const double border_color)
{
    cairo_set_line_width(cr, border_width);
    cairo_new_path(cr);

    cairo_move_to(cr, polyline.front().x(), polyline.front().y());
    for (size_t i = 1; i < polyline.size(); ++i)
    {
        cairo_line_to(cr, polyline[i].x(), polyline[i].y());
    }
    cairo_close_path(cr);

    cairo_set_source_rgba(cr, 0, 0, 0, fill_color);
    cairo_fill_preserve(cr);

    cairo_set_source_rgba(cr, 0, 0, 0, border_color);
    cairo_stroke(cr);
}

void drawPole(cairo_t* cr, const Eigen::Vector2i center, const double radius, const double border_width,
              const double fill_color, const double border_color)
{
    cairo_set_line_width(cr, border_width);
    cairo_new_path(cr);

    cairo_arc(cr, center.x(), center.y(), radius, 0, 2 * M_PI);

    cairo_set_source_rgba(cr, 0, 0, 0, fill_color);
    cairo_fill_preserve(cr);

    cairo_set_source_rgba(cr, 0, 0, 0, border_color);
    cairo_stroke(cr);
}

struct Pole
{
    bool reflective;
    double radius;
    Eigen::Vector2d origin;
};

struct FreeSpace
{
    std::vector<Eigen::Vector2d> points;
};

struct Polyline
{
    std::vector<Eigen::Vector2d> points;
};

struct World
{
    std::vector<FreeSpace> free_spaces;
    std::vector<Polyline> polylines;
    std::vector<Pole> poles;
};

World readWorldSDF(const std::string& world_sdf)
{
    LOG(INFO) << "Loading " << world_sdf;
    TiXmlDocument doc;
    doc.LoadFile(world_sdf.c_str());
    TiXmlHandle handle(&doc);
    TiXmlElement* sdf = handle.FirstChildElement("sdf").ToElement();
    CHECK(sdf != nullptr);

    const auto world = sdf->FirstChildElement("world");
    CHECK(world != nullptr);

    World result;

    for (const TiXmlElement* model : constIterate(world, "model"))
    {
        LOG(INFO) << "Loading model: " << model->Attribute("name");

        const Eigen::Isometry3d model_pose = getPose(model->FirstChildElement("pose")).cast<double>();

        unsigned int count = 0;

        for (const TiXmlElement* link : constIterate(model, "link"))
        {
            const std::string name = safe_string(link->Attribute("name"));
            const std::string layer = safe_string(link->Attribute("layer"));

            const Eigen::Isometry3d link_pose = model_pose * getPose(link->FirstChildElement("pose")).cast<double>();

            const auto collision = link->FirstChildElement("collision");
            const auto collision_geometry = collision ? collision->FirstChildElement("geometry") : nullptr;

            const auto polyline = collision_geometry ? collision_geometry->FirstChildElement("polyline") : nullptr;
            if (polyline)
            {
                const auto height = polyline->FirstChildElement("height");
                if (!height)
                    continue;

                const double height_value = std::stod(height->GetText());

                if (height_value <= 0)
                    continue;

                const std::vector<Eigen::Vector2d> points = getPolyline(polyline);
                const std::vector<Eigen::Vector2d> transformed_points = transformPoints(points, link_pose);

                count++;
                result.polylines.push_back(Polyline{transformed_points});
            }
            else
            {
                const auto cylinder = collision_geometry ? collision_geometry->FirstChildElement("cylinder") : nullptr;
                if (!cylinder)
                    continue;

                const auto radius = cylinder->FirstChildElement("radius");
                if (!radius)
                    continue;

                const auto laser_retro = collision->FirstChildElement("laser_retro");
                const double radius_value = std::stod(std::string(radius->GetText()));

                result.poles.push_back(Pole{laser_retro, radius_value, link_pose.translation().head<2>()});
            }
        }

        LOG(INFO) << "Loaded " << count << " links from: " << model->Attribute("name");
    }

    return result;
}

cairo_surface_t* makeSurface(const Eigen::Vector2i& map_size, const World& world, const Eigen::Vector2d& map_origin,
                             const double map_resolution)
{
    cairo_surface_t* surface = cairo_image_surface_create(cairo_format_t::CAIRO_FORMAT_A8, map_size.x(), map_size.y());

    const double collision_value = 100.0 / 255.0;
    const double free_value = 0.0;
    const double unknown_value = 1.0;

    cairo_t* cr = cairo_create(surface);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);

    // If no free space is provided, then assume default value is free and give a warning
    double default_value = unknown_value;
    if (world.free_spaces.size() == 0)
    {
        default_value = free_value;
        LOG(WARNING) << "No free space polygons provided. Setting default state to free. This may slow down your map.";
    }

    // Set everything to unknown
    cairo_set_source_rgba(cr, 0, 0, 0, default_value);
    cairo_set_line_width(cr, 1.0);
    cairo_rectangle(cr, 0, 0, map_size.x(), map_size.y());
    cairo_fill(cr);

    for (const FreeSpace& fs : world.free_spaces)
    {
        const std::vector<Eigen::Vector2i> polyline_cells = worldToMap(fs.points, map_origin, map_resolution);
        drawPolyline(cr, polyline_cells, 2.0, free_value, free_value);
    }

    for (const Polyline& pl : world.polylines)
    {
        const std::vector<Eigen::Vector2i> polyline_cells = worldToMap(pl.points, map_origin, map_resolution);
        drawPolyline(cr, polyline_cells, 2.0, unknown_value, collision_value);
    }

    for (const Pole& pole : world.poles)
    {
        const Eigen::Vector2i cell = worldToMap(pole.origin, map_origin, map_resolution);
        const double radius_px = pole.radius / map_resolution;
        drawPole(cr, cell, radius_px, 3.0, unknown_value, collision_value);
    }

    return surface;
}

int main(int argc, char** argv)
{
    FLAGS_alsologtostderr = true;
    google::InitGoogleLogging(argv[0]);

    std::string configuration_directory;
    std::string pbstream = "map.pbstream";
    std::string map_png;
    std::string map_origin_arg = "0,0";
    std::string map_size_arg = "64,64";
    std::string world_sdf;
    double map_resolution = 0.02;

    po::options_description desc("Options");
    try
    {
        // clang-format off
        desc.add_options()
        ("help,h", "Help Screen")
        ("configuration_directory", po::value<std::string>(&configuration_directory)->required(), "Cartographer configuration directory")
        ("pbstream", po::value<std::string>(&pbstream), "Pbstream destination")
        ("map_png", po::value<std::string>(&map_png), "Map PNG destination")
        ("map_origin", po::value<std::string>(&map_origin_arg), "Map origin in meters 'x,y'")
        ("map_size", po::value<std::string>(&map_size_arg), "Map size in meters 'x,y'")
        ("world_sdf", po::value<std::string>(&world_sdf)->required(), "World SDF file")
        ("map_resolution", po::value<double>(&map_resolution), "Map resolution")
        ("submap_location", po::value<std::vector<std::string>>()->multitoken()->required(), "Cartographer Submap Location")
        ("free_space", po::value<std::vector<std::string>>()->multitoken(), "Free space polygon");
        // clang-format on

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "display_pattern: " << std::endl << desc << std::endl;
            return EXIT_SUCCESS;
        }

        po::notify(vm);

        Eigen::Vector2i map_size = Eigen::Vector2i::Zero();
        {
            std::vector<std::string> map_size_strs;
            boost::split(map_size_strs, map_size_arg, boost::is_any_of(","));
            CHECK(map_size_strs.size() == 2);
            map_size.x() = static_cast<int>(std::stod(map_size_strs[0]) / map_resolution);
            map_size.y() = static_cast<int>(std::stod(map_size_strs[1]) / map_resolution);
        }

        Eigen::Vector2d map_origin = Eigen::Vector2d::Zero();
        {
            std::vector<std::string> map_origin_strs;
            boost::split(map_origin_strs, map_origin_arg, boost::is_any_of(","));
            CHECK(map_origin_strs.size() == 2);
            map_origin.x() = std::stod(map_origin_strs[0]);
            map_origin.y() = std::stod(map_origin_strs[1]);
        }

        
        LOG(INFO) << "loading sdf";
        World world = readWorldSDF(world_sdf);
        LOG(INFO) << "finished loading sdf";

        // Add free spaces
        if (!vm["free_space"].empty())
        {
            const std::vector<std::string> free_spaces = vm["free_space"].as<std::vector<std::string>>();

            // Each element in free_spaces is a polygon in the form x1, y1, x2, y2, x3, y3...
            for (size_t i = 0; i < free_spaces.size(); ++i)
            {
                const std::string& free_space = free_spaces[i];

                std::vector<std::string> strs;
                boost::split(strs, free_space, boost::is_any_of(","));

                // Make sure we have an even number of numbers
                CHECK(strs.size() % 2 == 0);

                std::vector<Eigen::Vector2d> points;
                for (size_t j = 0; j < strs.size(); j+=2)
                {
                    points.push_back(Eigen::Vector2d(std::stod(strs[j]), std::stod(strs[j+1])));
                }
                world.free_spaces.push_back(FreeSpace{points});
            }
        }

        cairo_surface_t* surface = makeSurface(map_size, world, map_origin, map_resolution);
        LOG(INFO) << "Writing png to: " << map_png;
        cairo_surface_write_to_png(surface, map_png.c_str());

        auto file_resolver = absl::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
        const std::string code = file_resolver->GetFileContentOrDie("cartographer.lua");
        auto lua_parameter_dictionary =
            cartographer::common::LuaParameterDictionary::NonReferenceCounted(code, std::move(file_resolver));
        const auto map_builder_options = cartographer::mapping::CreateMapBuilderOptions(
            lua_parameter_dictionary->GetDictionary("map_builder").get());
        const auto trajectory_builder_options = cartographer::mapping::CreateTrajectoryBuilderOptions(
            lua_parameter_dictionary->GetDictionary("trajectory_builder").get());

        const std::vector<std::string> submap_locations = vm["submap_location"].as<std::vector<std::string>>();

        cartographer::mapping::PoseGraph2D pose_graph(map_builder_options.pose_graph_options());
        pose_graph.FreezeTrajectory(0);

        // Cairo can sometimes create memory-aligned rows that are larger than the image width, so when accessing
        // raw pixel data, we need to multiply by the stride instead of the image width
        const unsigned int stride = cairo_image_surface_get_stride(surface);

        for (size_t i = 0; i < submap_locations.size(); ++i)
        {
            const std::string& submap_location = submap_locations[i];

            std::vector<std::string> strs;
            boost::split(strs, submap_location, boost::is_any_of(","));
            CHECK(strs.size() == 4);

            const double submap_center_x = std::stod(strs[0]);
            const double submap_center_y = std::stod(strs[1]);
            const double submap_size_x = std::stod(strs[2]);
            const double submap_size_y = std::stod(strs[3]);

            const double submap_tl_x = submap_center_x - submap_size_x / 2.0;
            const double submap_tl_y = submap_center_y - submap_size_y / 2.0;

            const double submap_max_x = submap_center_x + submap_size_x / 2.0;
            const double submap_max_y = submap_center_y + submap_size_y / 2.0;

            const int submap_cells_offset_x = static_cast<int>(submap_tl_x / map_resolution);
            const int submap_cells_offset_y = static_cast<int>(submap_tl_y / map_resolution);

            const int submap_cells_x = static_cast<int>(submap_size_x / map_resolution);
            const int submap_cells_y = static_cast<int>(submap_size_y / map_resolution);

            cartographer::mapping::ValueConversionTables conversion_tables;

            // Cartographer grid indexing is row-major, so cell indexes are given as (row, column)
            // Index (0, 0) refers to the top right corner, so both X and Y need to be mirrored
            auto grid = absl::make_unique<cartographer::mapping::ProbabilityGrid>(
                cartographer::mapping::MapLimits(
                    map_resolution, Eigen::Vector2d(submap_max_x + map_origin.x(), submap_max_y + map_origin.y()),
                    cartographer::mapping::CellLimits(submap_cells_y, submap_cells_x)),
                &conversion_tables);

            for (int y = submap_cells_offset_y; y < submap_cells_offset_y + submap_cells_y; y++)
            {
                if (y < 0 || y >= map_size.y())
                    continue;

                for (int x = submap_cells_offset_x; x < submap_cells_offset_x + submap_cells_x; x++)
                {
                    if (x < 0 || x >= map_size.x())
                        continue;

                    uint8_t* pixel_data = reinterpret_cast<uint8_t*>(cairo_image_surface_get_data(surface));
                    const unsigned int idx = static_cast<unsigned int>(y * stride + x);
                    const uint8_t px = pixel_data[idx];

                    const int submap_x = x - submap_cells_offset_x;
                    const int submap_y = y - submap_cells_offset_y;

                    CHECK(submap_x >= 0 && submap_x < submap_cells_x);
                    CHECK(submap_y >= 0 && submap_y < submap_cells_y);

                    const int mirrored_x = submap_cells_x - submap_x - 1;
                    const int mirrored_y = submap_cells_y - submap_y - 1;

                    CHECK(mirrored_x >= 0 && mirrored_x < submap_cells_x);
                    CHECK(mirrored_y >= 0 && mirrored_y < submap_cells_y);

                    if (px == 100)
                    {
                        grid->SetProbability({mirrored_y, mirrored_x}, 1.f);
                    }
                    else if (px < 100)
                    {
                        grid->SetProbability({mirrored_y, mirrored_x}, 0.f);
                    }
                }
            }
            grid->FinishUpdate();

            const Eigen::Vector2f origin = {static_cast<float>(submap_center_x + map_origin.x()),
                                            static_cast<float>(submap_center_y + map_origin.y())};
            cartographer::mapping::Submap2D submap(
                origin, std::move(grid), &conversion_tables,
                trajectory_builder_options.trajectory_builder_2d_options().submaps_options());
            std::vector<cartographer::mapping::CircleFeature> cf;
            for (const Pole& p : world.poles)
            {
                if (p.reflective)
                {
                    cartographer::mapping::CircleFeature f;
                    f.keypoint.position.x() = static_cast<float>(p.origin.x());
                    f.keypoint.position.y() = static_cast<float>(p.origin.y());
                    f.keypoint.position.z() = 0.0;
                    f.fdescriptor.score = 0.0;
                    f.fdescriptor.radius = static_cast<float>(p.radius);
                    
                    if ((p.origin.x() >= submap_tl_x && p.origin.x() <= submap_max_x) &&
                        (p.origin.y() >= submap_tl_y && p.origin.y() <= submap_max_y))
                    {
                        cf.push_back(f);
                    }
                }
            }
            submap.SetCircleFeatures(cf);
            submap.Finish();

            cartographer::transform::Rigid3d global_submap_pose(
                cartographer::transform::Rigid3d::Vector(static_cast<double>(origin.x()),
                                                         static_cast<double>(origin.y()), 0),
                cartographer::transform::Rigid3d::Quaternion(1, 0, 0, 0));

            LOG(INFO) << "Adding Submap " << i << " at " << global_submap_pose;

            cartographer::mapping::proto::Node node;
            node.mutable_node_id()->set_trajectory_id(0);
            node.mutable_node_id()->set_node_index(static_cast<int>(i));
            *node.mutable_node_data()->mutable_local_pose() = cartographer::transform::ToProto(global_submap_pose);

            cartographer::mapping::proto::Submap submap_proto = submap.ToProto(true);
            submap_proto.mutable_submap_id()->set_trajectory_id(0);
            submap_proto.mutable_submap_id()->set_submap_index(static_cast<int>(i));

            pose_graph.AddSubmapFromProto(global_submap_pose, submap_proto);
            pose_graph.AddNodeFromProto(global_submap_pose, node);
        }

        pose_graph.FinishTrajectory(0);

        cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds t_opt;
        *t_opt.mutable_trajectory_builder_options() = trajectory_builder_options;

        std::vector<cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds> t_opts;
        t_opts.push_back(t_opt);

        LOG(INFO) << "Writing pbstream to: " << pbstream;
        cartographer::io::ProtoStreamWriter writer(pbstream);
        cartographer::io::WritePbStream(pose_graph, t_opts, &writer, true);
        writer.Close();
    }
    catch (const po::error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << desc << std::endl;
        return EXIT_FAILURE;
    }
}
