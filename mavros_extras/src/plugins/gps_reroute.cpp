/**
 * @brief GPS Reroute MOCAP plugin
 * @file gps_reroute.cpp
 * @author Tarek Taha <tarek@tarektaha.com>
 *
 * @addtogroup plugin
 * @{
 */
/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/SwiftSpp.h>
namespace mavplugin {
/**
 * @brief GPSReRoutePlugin plugin
 *
 * Sends GPS data from an external source to FCU. Useful when using an external RTK unit without a driver support.
 */

class GPSReRoutePlugin : public MavRosPlugin
{
public:
  GPSReRoutePlugin() :
    gpsNh("~gps_reroute"),
    uas(nullptr)
  { }

  void initialize(UAS &uas_)
  {
    uas = &uas_;
    gpsNh.param("gps_fix_topic", gpsTopicName, std::string("gps_fix"));
    gpsFixSub = gpsNh.subscribe(gpsTopicName, 1, &GPSReRoutePlugin::gpsReRouteCallback, this);
  }

  const message_map get_rx_handlers()
  {
    return { /* Rx disabled */ };
  }

private:
  ros::NodeHandle gpsNh;
  ros::Subscriber gpsFixSub;
  UAS *uas;
  std::string gpsTopicName;

//  void gpsReRouteCallback(const sensor_msgs::NavSatFix::ConstPtr &gpsFix)
  void gpsReRouteCallback(const mavros_msgs::SwiftSpp::ConstPtr &gpsFix)
  {
    mavlink_message_t msg;
    mavlink_hil_gps_t pos;
    pos.time_usec = gpsFix->header.stamp.toNSec()*1000;
    pos.lat = gpsFix->latitude_s;
    pos.lon = gpsFix->longitude_s;
    pos.alt = gpsFix->height_s;
    pos.vel = 0;//sqrt(vn * vn + ve * ve);		// [cm/s]
    pos.vn =  gpsFix->vn;// [cm/s]
    pos.ve =  gpsFix->ve;// [cm/s]
    pos.vd =  gpsFix->vd;// [cm/s]
    pos.cog = 0;// [degrees * 100]
    pos.eph = gpsFix->hdop;
    pos.epv = gpsFix->vdop;
    pos.fix_type = 3;
    pos.satellites_visible = gpsFix->number_of_Sat;
    mavlink_msg_hil_gps_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                  pos.time_usec,
                                  pos.fix_type,
                                  pos.lat,
                                  pos.lon,
                                  pos.alt,
                                  pos.eph,
                                  pos.epv,
                                  pos.vel,
                                  pos.vn,
                                  pos.ve,
                                  pos.vd,
                                  pos.cog,
                                  pos.satellites_visible);
    UAS_FCU(uas)->send_message(&msg);
  }
};
}	// namespace mavros

PLUGINLIB_EXPORT_CLASS(mavplugin::GPSReRoutePlugin, mavplugin::MavRosPlugin)
