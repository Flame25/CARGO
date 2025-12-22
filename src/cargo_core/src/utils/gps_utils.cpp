#include "cargo_core/utils/gps_utils.hpp"

namespace cargo {
namespace gps {

// SET ORIGIN
void GpsHandler::setOrigin(const double &lat0, const double &lon0,
                           const double &h0) {
    if (this->is_origin_set_) {
        throw OriginAlreadySet();
    }
    this->Reset(lat0, lon0, h0);
    this->is_origin_set_ = true;
}

void GpsHandler::setOrigin(const sensor_msgs::msg::NavSatFix &fix) {
    this->setOrigin(fix.latitude, fix.longitude, fix.altitude);
}

void GpsHandler::setOrigin(const geographic_msgs::msg::GeoPoseStamped &gps) {
    this->setOrigin(gps.pose.position.latitude, gps.pose.position.longitude,
                    gps.pose.position.altitude);
}

// GET ORIGIN
void GpsHandler::getOrigin(double &rLat, double &rLon, double &rH) {
    if (!this->is_origin_set_) {
        throw OriginNonSet();
    }
    rLat = this->LatitudeOrigin();
    rLon = this->LongitudeOrigin();
    rH = this->HeightOrigin();
}

void GpsHandler::getOrigin(geographic_msgs::msg::GeoPoseStamped &gps) {
    double lat, lon, alt;
    this->getOrigin(lat, lon, alt);
    // gps.header.stamp = 0;
    gps.header.frame_id = global_frame;
    gps.pose.position.latitude = lat;
    gps.pose.position.longitude = lon;
    gps.pose.position.altitude = alt;
}

// LAT LON ---> LOCAL
void GpsHandler::LatLon2Local(const double &lat, const double &lon,
                              const double &h, double &rX, double &rY,
                              double &rZ) {
    if (!this->is_origin_set_) {
        throw OriginNonSet();
    }
    this->Forward(lat, lon, h, rX, rY, rZ);
}

void GpsHandler::LatLon2Local(const sensor_msgs::msg::NavSatFix &fix,
                              double &rX, double &rY, double &rZ) {
    this->LatLon2Local(fix.latitude, fix.longitude, fix.altitude, rX, rY, rZ);
}

void GpsHandler::LatLon2Local(const geographic_msgs::msg::GeoPoseStamped &gps,
                              double &rX, double &rY, double &rZ) {
    this->LatLon2Local(gps.pose.position.latitude, gps.pose.position.longitude,
                       gps.pose.position.altitude, rX, rY, rZ);
}

void GpsHandler::LatLon2Local(const double &lat, const double &lon,
                              const double &h,
                              geometry_msgs::msg::PoseStamped &ps) {
    double x, y, z;
    this->LatLon2Local(lat, lon, h, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = this->local_frame_;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsHandler::LatLon2Local(const sensor_msgs::msg::NavSatFix &fix,
                              geometry_msgs::msg::PoseStamped &ps) {
    this->LatLon2Local(fix.latitude, fix.longitude, fix.altitude, ps);
}

void GpsHandler::LatLon2Local(const geographic_msgs::msg::GeoPoseStamped &gps,
                              geometry_msgs::msg::PoseStamped &ps) {
    this->LatLon2Local(gps.pose.position.latitude, gps.pose.position.longitude,
                       gps.pose.position.altitude, ps);
}

// LOCAL ---> LAT LON
void GpsHandler::Local2LatLon(const double &x, const double &y, const double &z,
                              double &rLat, double &rLon, double &rH) {
    if (!this->is_origin_set_) {
        throw OriginNonSet();
    }
    this->Reverse(x, y, z, rLat, rLon, rH);
}

void GpsHandler::Local2LatLon(const double &x, const double &y, const double &z,
                              geographic_msgs::msg::GeoPoseStamped &gps) {
    double lat, lon, alt;
    this->Local2LatLon(x, y, z, lat, lon, alt);
    // gps.header.stamp = 0;
    gps.header.frame_id = global_frame;
    gps.pose.position.latitude = lat;
    gps.pose.position.longitude = lon;
    gps.pose.position.altitude = alt;
}

void GpsHandler::Local2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                              double &rLat, double &rLon, double &rH) {
    this->Local2LatLon(ps.pose.position.x, ps.pose.position.y,
                       ps.pose.position.z, rLat, rLon, rH);
}

void GpsHandler::Local2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                              geographic_msgs::msg::GeoPoseStamped &gps) {
    this->Local2LatLon(ps.pose.position.x, ps.pose.position.y,
                       ps.pose.position.z, gps);
}

// LAT LON ----> ECEF
void GpsHandler::LatLon2Ecef(const double &lat, const double &lon,
                             const double &h, double &rX, double &rY,
                             double &rZ) {
    earth.Forward(lat, lon, h, rX, rY, rZ);
}

void GpsHandler::LatLon2Ecef(const sensor_msgs::msg::NavSatFix &fix, double &rX,
                             double &rY, double &rZ) {
    GpsHandler::LatLon2Ecef(fix.latitude, fix.longitude, fix.altitude, rX, rY,
                            rZ);
}

void GpsHandler::LatLon2Ecef(const geographic_msgs::msg::GeoPoseStamped &gps,
                             double &rX, double &rY, double &rZ) {
    GpsHandler::LatLon2Ecef(gps.pose.position.latitude,
                            gps.pose.position.longitude,
                            gps.pose.position.altitude, rX, rY, rZ);
}

void GpsHandler::LatLon2Ecef(const double &lat, const double &lon,
                             const double &h,
                             geometry_msgs::msg::PoseStamped &ps) {
    double x, y, z;
    GpsHandler::LatLon2Ecef(lat, lon, h, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = global_frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsHandler::LatLon2Ecef(const sensor_msgs::msg::NavSatFix &fix,
                             geometry_msgs::msg::PoseStamped &ps) {
    GpsHandler::LatLon2Ecef(fix.latitude, fix.longitude, fix.altitude, ps);
}

void GpsHandler::LatLon2Ecef(const geographic_msgs::msg::GeoPoseStamped &gps,
                             geometry_msgs::msg::PoseStamped &ps) {
    GpsHandler::LatLon2Ecef(gps.pose.position.latitude,
                            gps.pose.position.longitude,
                            gps.pose.position.altitude, ps);
}

// ECEF ---> LAT LON
void GpsHandler::Ecef2LatLon(const double &x, const double &y, const double &z,
                             double &rLat, double &rLon, double &rH) {
    earth.Reverse(x, y, z, rLat, rLon, rH);
}

void GpsHandler::Ecef2LatLon(const double &x, const double &y, const double &z,
                             geographic_msgs::msg::GeoPoseStamped &gps) {
    double lat, lon, alt;
    GpsHandler::Ecef2LatLon(x, y, z, lat, lon, alt);
    // gps.header.stamp = 0;
    gps.header.frame_id = global_frame;
    gps.pose.position.latitude = lat;
    gps.pose.position.longitude = lon;
    gps.pose.position.altitude = alt;
}

void GpsHandler::Ecef2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                             double &rLat, double &rLon, double &rH) {
    GpsHandler::Ecef2LatLon(ps.pose.position.x, ps.pose.position.y,
                            ps.pose.position.z, rLat, rLon, rH);
}

void Ecef2LatLon(const geometry_msgs::msg::PoseStamped &ps,
                 geographic_msgs::msg::GeoPoseStamped &gps) {
    GpsHandler::Ecef2LatLon(ps.pose.position.x, ps.pose.position.y,
                            ps.pose.position.z, gps);
}

} // namespace gps
} // namespace cargo
