#include "render.h"
#include "route_model.h"
#include "route_planner.h"
#include <fstream>
#include <io2d.h>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path) {
  std::ifstream is{path, std::ios::binary | std::ios::ate};
  if (!is)
    return std::nullopt;

  auto size = is.tellg();
  std::vector<std::byte> contents(size);

  is.seekg(0);
  is.read((char *)contents.data(), size);

  if (contents.empty())
    return std::nullopt;
  return std::move(contents);
}

int main(int argc, const char **argv) {
  std::string osm_data_file = "";
  if (argc > 1) {
    for (int i = 1; i < argc; ++i)
      if (std::string_view{argv[i]} == "-f" && ++i < argc)
        osm_data_file = argv[i];
  } else {
    std::cout << "To specify a map file use the following format: "
              << std::endl;
    std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
    osm_data_file = "../map.osm";
  }

  std::vector<std::byte> osm_data;

  if (osm_data.empty() && !osm_data_file.empty()) {
    std::cout << "Reading the data---> Debug" << std::endl;
    std::cout << "Reading OpenStreetMap data from the following file: "
              << osm_data_file << std::endl;
    auto data = ReadFile(osm_data_file);
    if (!data)
      std::cout << "Failed to read." << std::endl;
    else
      osm_data = std::move(*data);
  }

  // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
  // user input for these values using std::cin. Pass the user input to the
  // RoutePlanner object below in place of 10, 10, 90, 90.
  float start_x, start_y, end_x, end_y;

  std::cout
      << std::endl
      << "Enter the coordinates for the start and end point: " << std::endl
      << std::endl
      << "(note)The coordinate (0, 0) should roughly correspond with the "
      << std::endl
      << "lower left corner of the map, and (100, 100) with the upper right."
      << std::endl;

  std::cout << std::endl << "Example input 10 10 20 20" << std::endl;

  std::cout << std::endl
            << "Note that for some inputs, the nodes might be slightly "
            << std::endl
            << "off(within +-10% range) the edges of the map, and this is fine."
            << std::endl;

  std::cin >> start_x >> start_y >> end_x >> end_y;

  std::cout << "x1 " << start_x << std::endl
            << "y1: " << start_y << std::endl
            << "x2: " << end_x << std::endl
            << "y2: " << end_y << std::endl;

  // Build Model.
  RouteModel model{osm_data};

  //   // Create RoutePlanner object and perform A* search.
  RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
  route_planner.AStarSearch();

  std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n ";

  // Render results of search.
  Render render{model};

  auto display = io2d::output_surface{400,
                                      400,
                                      io2d::format::argb32,
                                      io2d::scaling::none,
                                      io2d::refresh_style::fixed,
                                      30};

  display.size_change_callback([](io2d::output_surface &surface) {
    surface.dimensions(surface.display_dimensions());
  });
  display.draw_callback(
      [&](io2d::output_surface &surface) { render.Display(surface); });
  display.begin_show();
}
