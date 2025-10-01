#include "gps_navigation_utils.hpp"
#include <iostream>
#include <fstream>
#include "kml/convenience/convenience.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/base/file.h"

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

    int cont = 0;
    kmldom::KmlPtr kml = kmldom::KmlFactory::GetFactory()->CreateKml();
    

    // Create a <Folder> and a ContainerSaver to write to it.
    FolderPtr folder = kmldom::KmlFactory::GetFactory()->CreateFolder();


    float lat,lon,alt;
    for  (string line; getline(plan_file, line);) {
        sscanf(line.c_str(), "%f,%f,%f", &lat,&lon,&alt);

        // Create placemark
        char buf[14];
        sprintf(buf, "WP%03d", cont++);
        
        cout << "Parsing element: " << buf << endl;

        PlacemarkPtr placemark = createPointPlacemarkAltitude(string(buf), lat, lon, alt, cont-1);
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
