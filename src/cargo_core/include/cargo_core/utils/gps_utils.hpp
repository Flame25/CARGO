#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <string>

#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace cargo {
namespace gps {

static const GeographicLib::Geocentric &earth =
    GeographicLib::Geocentric::WGS84();
static const char global_frame[] =
    "earth"; // wgs84 --> ROS REP105 Name Convention

class OriginNonSet : public std::runtime_error {
  public:
    OriginNonSet() : std::runtime_error("origin is not set") {}
};

class OriginAlreadySet : public std::runtime_error {
  public:
    OriginAlreadySet() : std::runtime_error("origin can only be set once") {}
};

class GpsHandler : private GeographicLib::LocalCartesian {
  public:
    /**
     * @brief Construct a new Gps Handler object based on WGS84 ellipsoid
     *
     */
    GpsHandler() : GeographicLib::LocalCartesian(earth) {}

    /**
     * @brief Construct a new Gps Handler object based on WGS84 ellipsoid with a
     * given origin
     *
     * @param lat0 Latitude at origin (degrees)
     * @param lon0 Longitude at origin (degrees)
     * @param h0 Altitude at origin (meters)
     */
    GpsHandler(double lat0, double lon0, double h0 = 0)
        : GeographicLib::LocalCartesian(lat0, lon0, h0, earth) {
        this->is_origin_set_ = true;
    }

    /****************************************************************************************
     *                                                                                      *
     *                                    ORIGIN             *
     *                                                                                      *
     ***************************************************************************************/
    void setOrigin(const double &lat0, const double &lon0,
                   const double &h0 = 0);
    void setOrigin(const sensor_msgs::msg::NavSatFix &fix);
    void setOrigin(const geographic_msgs::msg::GeoPoseStamped &gps);
    void getOrigin(double &rLat, double &rLon, double &rH);
    void getOrigin(geographic_msgs::msg::GeoPoseStamped &gps);

    /****************************************************************************************
     *                                                                                      *
     *                        Geodesic LLA to Local Cartesian             *
     *                                                                                      *
     ***************************************************************************************/
    void LatLon2Local(const double &lat, const double &lon, const double &h,
                      double &rX, double &rY, double &rZ);
    void LatLon2Local(const sensor_msgs::msg::NavSatFix &fix, double &rX,
                      double &rY, double &rZ);
    void LatLon2Local(const geographic_msgs::msg::GeoPoseStamped &gps,
                      double &rX, double &rY, double &rZ);
    void LatLon2Local(const double &lat, const double &lon, const double &h,
                      geometry_msgs::msg::PoseStamped &ps);
    void LatLon2Local(const sensor_msgs::msg::NavSatFix &fix,
                      geometry_msgs::msg::PoseStamped &ps);
    void LatLon2Local(const geographic_msgs::msg::GeoPoseStamped &gps,
                      geometry_msgs::msg::PoseStamped &ps);

    /****************************************************************************************
     *                                                                                      *
     *                        Local cartesian to Geodesic LLA             *
     *                                                                                      *
     ***************************************************************************************/
    void Local2LatLon(const double &x, const double &y, const double &z,
                      double &rLat, double &rLon, double &rH);
    void Local2LatLon(const double &x, const double &y, const double &z,
                      geographic_msgs::msg::GeoPoseStamped &gps);
    void Local2LatLon(const geometry_msgs::msg::PoseStamped &ps, double &rLat,
                      double &rLon, double &rH);
    void Local2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                      geographic_msgs::msg::GeoPoseStamped &gps);

    /****************************************************************************************
     *                                                                                      *
     *                      Geodesic LLA to Earth-Centered-Earth-Fixed
     *            *
     *                                                                                      *
     ***************************************************************************************/
    static void LatLon2Ecef(const double &lat, const double &lon,
                            const double &h, double &rX, double &rY,
                            double &rZ);
    static void LatLon2Ecef(const sensor_msgs::msg::NavSatFix &fix, double &rX,
                            double &rY, double &rZ);
    static void LatLon2Ecef(const geographic_msgs::msg::GeoPoseStamped &gps,
                            double &rX, double &rY, double &rZ);
    static void LatLon2Ecef(const double &lat, const double &lon,
                            const double &h,
                            geometry_msgs::msg::PoseStamped &ps);
    static void LatLon2Ecef(const sensor_msgs::msg::NavSatFix &fix,
                            geometry_msgs::msg::PoseStamped &ps);
    static void LatLon2Ecef(const geographic_msgs::msg::GeoPoseStamped &gps,
                            geometry_msgs::msg::PoseStamped &ps);

    /****************************************************************************************
     *                                                                                      *
     *                      Earth-Centered-Earth-Fixed to Geodesic LLA
     *            *
     *                                                                                      *
     ***************************************************************************************/
    static void Ecef2LatLon(const double &x, const double &y, const double &z,
                            double &rLat, double &rLon, double &rH);
    static void Ecef2LatLon(const double &x, const double &y, const double &z,
                            geographic_msgs::msg::GeoPoseStamped &gps);
    static void Ecef2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                            double &rLat, double &rLon, double &rH);
    static void Ecef2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                            geographic_msgs::msg::GeoPoseStamped &gps);

  private:
    const std::string local_frame_ =
        "map"; // local world fixed --> ROS REP105 Name Convention
    bool is_origin_set_ = false;
}; // GpsHandler

} // namespace gps
} // namespace cargo
