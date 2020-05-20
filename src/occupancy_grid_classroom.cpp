// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <vector>
#include <string>
#include <math.h>
// %EndTag(INCLUDES)%


class Sonar_beams
{
    public:
        
    double kobuki_x;
    double kobuki_y;

    //Distancia maxima para la deteccion de colisionesmetros
    double DEFAULT_BEAM_LENGTH = 2;

    //Espacio para markers
    std::vector<visualization_msgs::Marker> markers(0);


    visualization_msgs::Marker draw_beam(int id, double angle)
    {
        visualization_msgs::Marker beam;
        beam.header.frame_id = "/base_link";
        beam.header.stamp = ros::Time::now();
        beam.ns = "sonar";
        beam.id = id;
        beam.type = visualization_msgs::Marker::ARROW;
        beam.action = visualization_msgs::Marker::ADD;
        beam.pose.position.x = 0;
        beam.pose.position.y = 0;
        beam.pose.position.z = 0;
        beam.scale.x = DEFAULT_BEAM_LENGTH;
        beam.scale.y = 0.1;
        beam.scale.z = 0.1;
        
        beam.color.a = 1.0;
        beam.color.r = 0.0;
        beam.color.g = 0.0;
        beam.color.b = 1.0;
        
        //Deal with quaternions
        double this_is_needed_for_my_quaternion = std::sin(angle / 2);
        double z =  1 * this_is_needed_for_my_quaternion;
        double x_and_y = 0;
        double w = std::cos(angle / 2);
        this_is_needed_for_my_quaternion = sqrt(z*z + w*w);
        //Now save normalized values
        beam.pose.orientation.x = 0;
        beam.pose.orientation.y = 0;
        beam.pose.orientation.z = z/this_is_needed_for_my_quaternion;
        beam.pose.orientation.w = w/this_is_needed_for_my_quaternion;
        
        return beam;
    }

    int update_beam( visualization_msgs::Marker *beam, double beam_length)
    {
        beam -> header.stamp = ros::Time::now();
        beam -> pose.position.x = 0;
        beam -> pose.position.y = 0;
        beam -> pose.position.z = 0;
        
        //This should change in order to represent colissions
        beam -> scale.x = beam_length;
        
        return 0;
    }

    /*
    Check kobuki coordinates and angle in relation to rviz, 
    check a straigh lines for each beam and set the X scale of the beam 
    acordingly (to avoid colissions).
    */
    int sonar_update()
    {
        visualization_msgs::Marker *beam;
        double endpoint;
        
        
        // Now update each beam marker
        for(int i = 0; i <markers.size() ; ++i)
        {
            beam = &markers[i];
            
            //find endpoint for the current beam
            endpoint = DEFAULT_BEAM_LENGTH;
            
            update_beam(beam, endpoint);
        }
        
        return 0;
    }


    /*
    Recibe la posicion de la kobuki y un rayo, buscando colisiones desde la 
    posición de la kobuki y devuelve las coordenadas de la primera colisión encontrada
    ó las que corresponden a DEFAULT_BEAM_LENGTH
    */
    int make_beams(double angulo_sensores)
    {
        markers = std::vector<visualization_msgs::Marker> ( floor(6.2832 / angulo_sensores) );
        for(int i = 0; i < markers.size() ; ++i)
        {
            markers[i] = draw_beam(i, angulo_sensores*i);
        }
        return 0;
    }


    /** Receives the odometry data*/
    void receiveKobukiPosition(const nav_msgs::Odometry& odometry)
    {
        kobuki_x = odometry.pose.pose.position.x;
        kobuki_y = odometry.pose.pose.position.y;
    }


    int draw_beams()
    {
        ros::init(argc, argv, "collision_detection");
        ros::NodeHandle n;
        ros::Rate r(1);
        ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "sonar_marker", 1);
        ros::Subscriber sub = n.subscribe("/odom", 5, receiveKobukiPosition); 
                                                                        // Máximo 5 mensajes en la cola.
        // %EndTag(INIT)%
        int beam_number = 6;
        double beam_angle = 6.2832 / beam_number;
        make_beams(beam_angle);
        
        visualization_msgs::Marker *temp;
        
        if(markers.size() != 0)
        {
            temp = &markers[0];
            temp -> color.r = 1.0;
        }
        
        while (ros::ok())
        {
            sonar_update();
            for(int i = 0; i <markers.size() ; ++i)
            {
                //for each in markers
                temp = &markers[i];
                vis_pub.publish( *(temp) );
            }
            ros::spinOnce();
        }
    }
        
}

class classroom_map
{
    public:
    
}

/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

const int WIDTH = 22;      /// A lo largo del eje rojo x
const int HEIGHT = 17;     /// A lo largo del eje verde


visualization_msgs::Marker draw_table(int id, int pos_x, int pos_y, int size_x, int size_y){
    visualization_msgs::Marker table_marker;
    table_marker.header.frame_id = "/odom";
    table_marker.header.stamp = ros::Time::now();
    table_marker.ns = "tables";
    table_marker.id = id;
    table_marker.type = visualization_msgs::Marker::CUBE;
    table_marker.action = visualization_msgs::Marker::ADD;
    table_marker.pose.position.x = pos_x;
    table_marker.pose.position.y = pos_y;
    table_marker.pose.position.z = 0.25;
    table_marker.pose.orientation.x = 0.0;
    table_marker.pose.orientation.y = 0.0;
    table_marker.pose.orientation.z = 0.0;
    table_marker.pose.orientation.w = 1.0;
    table_marker.scale.x = size_x;
    table_marker.scale.y = size_y;
    table_marker.scale.z = 0.5;
    table_marker.color.a = 1.0; // Don't forget to set the alpha!
    table_marker.color.r = 1.0;
    table_marker.color.g = 1.0;
    table_marker.color.b = 1.0;
    
    return table_marker;
}

/** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
{
  for(int i = i1; i <= i2; i++)
  {
    for(int j = j1; j <= j2; j++)
    {
      data[i*WIDTH+j] = value;
    }
  }
}

/** Receives the message of the navigation goal from rviz. */
void receiveNavGoal(const geometry_msgs::PoseStamped& poseStamped)
{
  ROS_INFO("\nFrame: %s\nMove to: [%f, %f, %f] - [%f, %f, %f, %f]",
           poseStamped.header.frame_id.c_str(),
           poseStamped.pose.position.x,
           poseStamped.pose.position.y,
           poseStamped.pose.position.z,
           poseStamped.pose.orientation.x,
           poseStamped.pose.orientation.y,
           poseStamped.pose.orientation.z,
           poseStamped.pose.orientation.w);
}


// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "classroom_map");
  ros::NodeHandle n;
  ros::Rate r(1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher marker_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker_classroom", 1);
  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 5, receiveNavGoal); // Máximo 5 mensajes en la cola.
// %EndTag(INIT)%
  
  
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "object_marker", 1);

// %Tag(MAP_INIT)%
  nav_msgs::OccupancyGrid map;

  // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    map.header.frame_id = "/odom";
    map.header.stamp = ros::Time::now();   // No caduca

    map.info.resolution = 0.3;             // [m/cell]
    map.info.width = WIDTH;                // [cells]
    map.info.height = HEIGHT;              // [cells]
    map.info.origin.position.x = -1.35;
    map.info.origin.position.y = 0;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    //int8[] &_data = &map.data
    int size = WIDTH * HEIGHT;
    char* data = new char[size];
    for(int i = 0; i < size; i++) {
      data[i] = 0;
    }
    
    //Bordes
    fillRectangle(data, 0, 0, 0, WIDTH-1, 100);   // Renglón 0. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y.
    fillRectangle(data, 1, 0, HEIGHT-1, 0, 100);  // Columna 0. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X.
    fillRectangle(data, 0, WIDTH-1, HEIGHT-1, WIDTH-1, 100);
    fillRectangle(data, HEIGHT-1, 0, HEIGHT-1, WIDTH-1, 100);
    
    
    //Para cada mesa se guarda pos_x, pos_y, size_x, size_y
    std::vector<double> mesas(44, 0);
    
    //Puerta
    fillRectangle(data, 0, 3, 2, 5, 50);       // El origen está en la esquina inferior izquierda.
        //HEIGHT es el eje corto del salón y WIDTH el largo
        //en rviz HEIGHT es Y, WIDTH es X
        // 17 22
        // 5.07 6.56
        //cada mesa tiene (height_ini, width_ini, height_fin, width_fin)
    
    //Mesas tv
    fillRectangle(data, 1, 0, 3, 2, 100);
    mesas[0] = -1;
    mesas[1] = 1;
    mesas[2] = 0.8;
    mesas[3] = 0.8;
    
    fillRectangle(data, 6, 0, 9, 2, 100);
    mesas[4] = 1;
    mesas[5] = 6;
    mesas[6] = 1;
    mesas[7] = 1.5;
    
    
    //Mesas centro
    fillRectangle(data, 6, 6, 7, 11, 100);
    fillRectangle(data, 6, 12, 7, 17, 100);
    fillRectangle(data, 8, 6, 10, 11, 100);
    fillRectangle(data, 8, 12, 10, 17, 100);
    
    
    //Mesa-lado izq (Viendo hacia TV desde Pizarón)
    fillRectangle(data, 1, 6, 2, 11, 100);
    fillRectangle(data, 1, 12, 2, 15, 100);
    fillRectangle(data, 1, 16, 2, 19, 100);
    
    //Mesas derecha
    fillRectangle(data, 13, 0, 15, 5, 100);
    fillRectangle(data, 13, 15, 15, 20, 100);
    
    map.data = std::vector<int8_t>(data, data + size);

// %EndTag(MAP_INIT)%
    
    visualization_msgs::Marker temp;
    bool first = false;

  while (ros::ok())
  {
    if( first )
        marker_pub.publish(map);
    for(int i = 0; i < 44; i += 4)
    {
      temp = draw_table(i/4, mesas[i], mesas[i+1], mesas[i+2], mesas[i+3]);
      vis_pub.publish( temp );
    }

// %Tag(SLEEP_END)%
    ros::spinOnce();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
