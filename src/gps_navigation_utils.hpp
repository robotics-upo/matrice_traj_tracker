#pragma once

#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpUpload.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/base/file.h"
#include <GeographicLib/Geodesic.hpp>
#include <fstream>
#include <sstream>
#include <jsoncpp/json/json.h>

using kmldom::ContainerPtr;
using kmldom::ElementPtr;
using kmldom::FeaturePtr;
using kmldom::KmlPtr;
using kmldom::PlacemarkPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
using kmlengine::KmzFile;
using kmlengine::KmzFilePtr;
using std::cout;
using std::endl;

// Declare the types and functions defined in this file.
typedef std::vector<PlacemarkPtr> placemark_vector_t;
static FeaturePtr GetKmlFileRootFeature(const char* kmlfile);
static void SavePlacemarks(const FeaturePtr& feature,
                           placemark_vector_t* placemarks);

// Save the Feature to the placemarks vector if it is a Placemark.  If the
// Feature is a Container call ourselves recursively for each Feature in the
// Container.
static void SavePlacemarks(const FeaturePtr& feature,
                           placemark_vector_t* placemarks) {
  if (PlacemarkPtr placemark = kmldom::AsPlacemark(feature)) {
    placemarks->push_back(placemark);
  } else if (const ContainerPtr container = kmldom::AsContainer(feature)) {
    for (size_t i = 0; i < container->get_feature_array_size(); ++i) {
      SavePlacemarks(container->get_feature_array_at(i), placemarks);
    }
  }
}

// Return a FeaturePtr to the root Feature in the kmlfile.  If the kmlfile
// does not parse or has no root Feature then an empty FeaturePtr is returned.
static FeaturePtr GetKmlFileRootFeature(const char* kmlfile) {
  // Read the file.
  std::string file_data;
  if (!kmlbase::File::ReadFileToString(kmlfile, &file_data)) {
    cout << kmlfile << " read failed" << endl;
    return NULL;
  }

  // If the file was KMZ, extract the KML file.
  std::string kml;
  if (KmzFile::IsKmz(file_data)) {
    KmzFilePtr kmz_file = KmzFile::OpenFromString(kmlfile);
    if (!kmz_file) {
      cout << "Failed opening KMZ file" << endl;
      return NULL;
    }
    if (!kmz_file->ReadKml(&kml)) {
      cout << "Failed to read KML from KMZ" << endl;
      return NULL;
    }
  } else {
    kml = file_data;
  }

  // Parse it.
  std::string errors;
  KmlFilePtr kml_file = KmlFile::CreateFromParse(kml, &errors);
  if (!kml_file) {
    cout << errors << endl;
    return FeaturePtr();
  }

  // Get the root
  return kmlengine::GetRootFeature(kml_file->get_root());
}

// This function object is used by STL sort() to alphabetize Placemarks
// by <name>.
struct ComparePlacemarks
  : public
      std::binary_function<const PlacemarkPtr&, const PlacemarkPtr&, bool> {
  bool operator()(const PlacemarkPtr& a, const PlacemarkPtr& b) {
    return a->get_name() < b->get_name();
  }
};

inline void setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = 5;
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

inline dji_sdk::MissionWaypointTask getPlanFromKML(const std::string &filename) {
  placemark_vector_t placemark_vector;

  dji_sdk::MissionWaypointTask ret;
  setWaypointInitDefaults(ret);

  SavePlacemarks(GetKmlFileRootFeature(filename.c_str()), &placemark_vector);
  sort(placemark_vector.begin(), placemark_vector.end(), ComparePlacemarks());
  for (size_t i = 0; i < placemark_vector.size(); ++i) {
    dji_sdk::MissionWaypoint w;
    auto g = placemark_vector[i]->get_geometry();
    auto p = kmldom::AsPoint(g);
    kmldom::CoordinatesPtr c;
    if (p) {
        c = p->get_coordinates();
    }
    auto ls = kmldom::AsLineString(g);
    if (ls) {
        c = ls->get_coordinates();
    }
    cout << "Getting coordinates from placemark: " << placemark_vector[i]->get_id() << " " << placemark_vector[i]->get_name() << endl;
    for(int j = 0; j < c->get_coordinates_array_size(); j++) {
        dji_sdk::MissionWaypoint w;
        kmlbase::Vec3 coord = c->get_coordinates_array_at(j);
        w.altitude = coord.get_altitude();
        w.longitude = coord.get_longitude();
        w.latitude = coord.get_latitude();
        std::cout << "Lat: " << w.latitude << " Lng: " << w.longitude << " Alt: " << w.altitude << std::endl;
        ret.mission_waypoint.push_back(w);
    }

    cout << endl;
  }

  if(!ret.mission_waypoint.empty()){
    cout << "Distances between waypoints:" << endl;
    for(int j = 1; j < ret.mission_waypoint.size(); ++j) {
        const auto& w1 = ret.mission_waypoint[j - 1];
        const auto& w2 = ret.mission_waypoint[j];
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        double distance;
        geod.Inverse(w1.latitude, w1.longitude, w2.latitude, w2.longitude, distance);
        cout << "Distance from WP" << j - 1 << " to WP" << j << ": " << distance << " m" << endl;
    }
    cout << endl;

    cout << "Distances from WP home:" << endl;
    const auto& wphome = ret.mission_waypoint[0];
    for(int j = 1; j < ret.mission_waypoint.size(); ++j) {
        const auto& w = ret.mission_waypoint[j];
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        double distance;
        geod.Inverse(wphome.latitude, wphome.longitude, w.latitude, w.longitude, distance);
        cout << "Distance from WP" << j << " to home: " << distance << " m" << endl;
    }
    cout << endl;
  }
  return ret;
}

inline dji_sdk::MissionWaypointTask getPlanFromJson(const std::string &filename) {
  dji_sdk::MissionWaypointTask ret;
  setWaypointInitDefaults(ret);

  std::ifstream plan_file(filename);
  if (!plan_file.is_open()) {
    std::cerr << "Error: Could not open JSON file " << filename << std::endl;
    return ret;
  }

  Json::Value plan;
  plan_file >> plan;
  auto &waypoints = plan["waypoints"];

  for (auto &wp : waypoints) {
    if(wp.size() >= 3) {
      dji_sdk::MissionWaypoint w;
      w.latitude = wp[0].asDouble();
      w.longitude = wp[1].asDouble();
      w.altitude = wp[2].asDouble();
      ret.mission_waypoint.push_back(w);
      std::cout << "JSON WP -> Lat: " << w.latitude << " Lng: " << w.longitude << " Alt: " << w.altitude << std::endl;
    }
  }
  return ret;
}

inline dji_sdk::MissionWaypointTask getPlanFromCSV(const std::string &filename, bool &do_takeoff, bool &do_land) {
  dji_sdk::MissionWaypointTask ret;
  setWaypointInitDefaults(ret);

  std::ifstream plan_file(filename);
  if (!plan_file.is_open()) {
    std::cerr << "Error: Could not open CSV file " << filename << std::endl;
    return ret;
  }

  std::string line;
  while (std::getline(plan_file, line)) {
    std::stringstream ss(line);
    std::string action_type;
    
    // Read the first column (Action type)
    getline(ss, action_type, ',');

    // Set flags based on the action type
    if (action_type == "TAKEOFF") {
      do_takeoff = true;
      std::cout << "Command found: TAKEOFF (Parsing coords as initial waypoint)" << std::endl;
    } else if (action_type == "LAND") {
      do_land = true;
      std::cout << "Command found: LAND (Parsing coords as final waypoint)" << std::endl;
    }

    // Process coordinates
    if (action_type == "WAYPOINT" || action_type == "TAKEOFF" || action_type == "LAND") {
      dji_sdk::MissionWaypoint w;
      std::string substr;

      getline(ss, substr, ',');
      w.latitude = std::stof(substr);

      getline(ss, substr, ',');
      w.longitude = std::stof(substr);

      getline(ss, substr, ',');
      w.altitude = std::stof(substr);

      // Default waypoint values
      w.damping_distance = 1.0; 
      w.target_yaw = 0;
      w.target_gimbal_pitch = 0;
      w.turn_mode = 0;
      w.has_action = 0;

      ret.mission_waypoint.push_back(w);
    }
  }
  return ret;
}