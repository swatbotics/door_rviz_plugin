/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
   Modified by: Will Schneider <wschnei1@swarthmore.edu>
 * Date: August 14, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DOOR_RVIZ_PLUGIN_H
#define DOOR_RVIZ_PLUGIN_H

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QMouseEvent>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QImage>
#include <QByteArray>
#include <QLabel>
#include <QPushButton>

#include <vector>

#include <rviz/panel.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <hubo.h>
#include <hubo-jointparams.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

namespace door_plugin_space
{

class DoorRvizPlugin;


// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
class DoorRvizPlugin: public QTabWidget
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  DoorRvizPlugin( QWidget* parent = 0 );
  ~DoorRvizPlugin();
  void image_cb( const sensor_msgs::Image& image);
  void draw_cb( const geometry_msgs::Polygon& door_rect);
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  ros::Subscriber door_sub;
  ros::Publisher door_pub;
  ros::Publisher handle_pub;
  QByteArray image_bytes;
  QImage display_image;
  QPolygon door;

  QLabel* display_label;
  QPushButton* sample_button;


  // Slots will be "connected" to signals in order to respond to user events
protected:
  ros::Publisher doorPoints;
  ros::Publisher handlePoints;
  void paintEvent( QPaintEvent* event );
  void mousePressEvent( QMouseEvent* event );

signals:
  void outputDoorPoint(int x, int y);
  void outputHandlePoint(int x, int y);

protected Q_SLOTS:

  void addDoorPoint(int x, int y);
  void addHandlePoint(int x, int y);
  void sampleButtonClicked();


private:

  ///////////////
  
};


class DoorRvizPanel : public rviz::Panel
{
Q_OBJECT
public:
    DoorRvizPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.

private:

    DoorRvizPlugin* content;

};

} // end namespace rviz_plugin_tutorials


#endif // TELEOP_PANEL_H
