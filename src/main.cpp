#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

class InitData {
    public:
        float StartX() { return start_x; };
        void SetStartX (float x) {
            if (x >=0 && x <= 100) start_x = x;            
        }

        float StartY() { return start_y; };
        void SetStartY (float y) {
            if (y >=0 && y <= 100) start_y = y;
        }

        float EndX() { return end_x; }
        void SetEndX (float x) {
            if (x >=0 && x <= 100) end_x = x;
        }

        float EndY() { return end_y; }
        void SetEndY (float y) {
            if (y >=0 && y <= 100) end_y = y;
        }
    
    private:
        float start_x = 10;
        float start_y = 10;
        float end_x = 90;
        float end_y = 90;
};

/**
 * Calls for user input to set InitData
 */   
void SetInitData(InitData * initData) {
    float input_value;
    
    std::cout << "Starting postion X Value: ";
    std::cin >> input_value;
    initData->SetStartX(input_value);

    std::cout << "Starting postion Y Value: ";
    std::cin >> input_value;
    initData->SetStartY(input_value);

    std::cout << "Ending postion X Value: ";
    std::cin >> input_value;
    initData->SetEndX(input_value);

    std::cout << "Ending postion Y Value: "; 
    std::cin >> input_value;
    initData->SetEndY(input_value);    
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    //Intialize start and end node x,y
    InitData initData;        
    SetInitData(&initData);  
  

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, initData.StartX(), initData.StartY(), initData.EndX(), initData.EndY()};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
