#include "gps_navigation_utils.hpp"
#include <iostream>
#include <fstream>
#include "kml/convenience/convenience.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/base/file.h"

#include <jsoncpp/json/json.h>

using kmlbase::File;
using kmlconvenience::CreatePointPlacemark;
using kmldom::ContainerPtr;
using kmldom::FolderPtr;
using kmldom::KmlFactory;
using kmldom::KmlPtr;
using kmldom::PlacemarkPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;

using namespace std;

PlacemarkPtr createPointPlacemarkAltitude(const string &name, double lat, double lon, double alt, int id);

int main(int argc, char **argv) {

    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_json_file> <output_kml_file>\n";
        return -1;
    }

    ifstream plan_file(argv[1]);
    Json::Value plan;
    plan_file >> plan;

    auto &waypoints = plan["waypoints"];



    int cont = 0;
    kmldom::KmlPtr kml = kmldom::KmlFactory::GetFactory()->CreateKml();
    

    // Create a <Folder> and a ContainerSaver to write to it.
    FolderPtr folder = kmldom::KmlFactory::GetFactory()->CreateFolder();



    for (auto &wp:waypoints) {
        std::vector<double> coords;
        for (auto &coord:wp) {
            coords.push_back(coord.asDouble());
        }
        // Create placemark
        string s("WP");
        s.append(to_string(cont++));
        
        cout << "Parsing element: " << s << endl;

        PlacemarkPtr placemark = createPointPlacemarkAltitude(s, coords[0], coords[1], coords[2], cont-1);
        folder->add_feature(placemark);
    }

    kml->set_feature(folder);
    // Serialize to XML
    std::string xml = kmldom::SerializePretty(kml);

    return File::WriteStringToFile(xml, argv[2]) ? 1 : 0;
}

PlacemarkPtr createPointPlacemarkAltitude(const string &name, double lat, double lon, double alt, int id) {
    KmlFactory* factory = KmlFactory::GetFactory();
    PlacemarkPtr placemark = factory->CreatePlacemark();
    placemark->set_name(name);
    kmlbase::Vec3 vec(lon, lat, alt);
    placemark->set_geometry(kmlconvenience::CreatePointFromVec3(vec));
    placemark->set_id("wp_"+to_string(id));

    return placemark;
}
