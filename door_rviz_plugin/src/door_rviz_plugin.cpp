#include "door_rviz_plugin.h"
//#include "FlowLayout.h"
#include <QLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace door_plugin_space
{




void DoorRvizPlugin::image_cb (const sensor_msgs::Image& input_image)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::RGB8);
    cv::Mat data = cv_ptr->image;
    image_bytes = QByteArray((const char*)data.data, data.rows*data.cols*3);
    display_image = QImage((uchar*)image_bytes.data(), data.cols, data.rows, QImage::Format_RGB888);
    //display_label->setPixmap(QPixmap::fromImage(display_image));
    //display_label->setMinimumSize(display_image.size());
    //display_label->setMaximumSize(display_image.size());
    update();
}

void DoorRvizPlugin::draw_cb (const geometry_msgs::Polygon& door_rect)
{
    door.clear();
    QPoint q ( door_rect.points[0].x, door_rect.points[0].y);
    for ( int i = 0; i < door_rect.points.size(); i++ )
    {
        QPoint p(door_rect.points[i].x, door_rect.points[i].y);
        door.append(p);
    }
    door.append(q);
    update();
}

DoorRvizPanel::DoorRvizPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new DoorRvizPlugin;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

DoorRvizPlugin::~DoorRvizPlugin()
{
}


DoorRvizPlugin::DoorRvizPlugin(QWidget *parent)
    : QTabWidget(parent)
{
    const std::string image_name = "door/image";
    uint32_t buf_size = 1;
    image_sub = nh.subscribe(image_name, buf_size, &DoorRvizPlugin::image_cb, this);
    door_sub = nh.subscribe("drawPoints", 1, &DoorRvizPlugin::draw_cb, this);
    
    door_pub = nh.advertise<geometry_msgs::Point>("doorPoints", 1);
    handle_pub = nh.advertise<geometry_msgs::Point>("handlePoints", 1);

    connect(this, SIGNAL(outputDoorPoint(int, int)), this, SLOT(addDoorPoint(int, int)));
    connect(this, SIGNAL(outputHandlePoint(int, int)), this, SLOT(addHandlePoint(int, int)));

    display_label = new QLabel(this);

    sample_button = new QPushButton("Click me", this);

    connect(sample_button, SIGNAL(clicked()), this, SLOT(sampleButtonClicked()));

    QHBoxLayout* h = new QHBoxLayout(this);

    QVBoxLayout* v1 = new QVBoxLayout();

    v1->addWidget(display_label);
    v1->addStretch();
    
    QVBoxLayout* v2 = new QVBoxLayout();
    v2->addWidget(sample_button);
    v2->addStretch();
    
    h->addLayout(v1);    
    h->addLayout(v2);    

}

void DoorRvizPlugin::sampleButtonClicked() {
    std::cout << "YOU CLICKED ME!\n";
}


void DoorRvizPlugin::paintEvent( QPaintEvent* event )
{
    QPainter painter;
    painter.begin(this);
    //QRect bg = QRect(0, 0, display_image.width(), display_image.height());
    painter.drawImage( QPoint(0,0), display_image );
    painter.setPen(QColor("red"));
    
    //draw the points and lines
    painter.drawPolyline(door);
    painter.end();
}

void DoorRvizPlugin::mousePressEvent( QMouseEvent* event )
{


    QPoint pos = event->pos();
    QPoint label_pos = display_label->mapFromParent(pos);

    if (display_image.valid(label_pos)) {

        if (event->button() == Qt::LeftButton)
        {
            Q_EMIT outputDoorPoint( event->x(), event->y() );
        }
        else if (event->button() == Qt::RightButton)
        {
            Q_EMIT outputHandlePoint( event->x(), event->y() );
        }
        update();
    }
}

void DoorRvizPlugin::addDoorPoint(int x, int y)
{
    geometry_msgs::Point output;
    output.x = x;
    output.y = y;
    door_pub.publish(output);
}

void DoorRvizPlugin::addHandlePoint(int x, int y)
{
    geometry_msgs::Point output;
    output.x = x;
    output.y = y;
    handle_pub.publish(output);
}


} // End rviz_plugin_space




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( door_plugin_space::DoorRvizPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( door_plugin_space::DoorRvizPlugin, QTabWidget )
