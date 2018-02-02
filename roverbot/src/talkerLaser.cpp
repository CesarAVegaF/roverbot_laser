#include <bits/stdc++.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "roverbot/Obstacles.h"
#include "roverbot/Point.h"

#define PI 3.14159265
using namespace std;
unsigned int num_points = 100;

float xBot = 6;
float yBot = 0;
float zBot, normaBot;
float norma, prodIn, cosA;
float xCloud, yCloud, zCloud;
vector<double> vec;



class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<roverbot::Obstacles>("roverbot/obstacles",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(10.0))){
        return;
    }

    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    scan_pub_.publish(printPoints(cloud));

    ROS_INFO("I published something!");
  }

  /**
  * Funcion para el tratamiento de la informacion de la nube de puntos
  */
  roverbot::Obstacles printPoints(sensor_msgs::PointCloud cloud)
  {

    num_points = cloud.points.size();  // número de puntos en la lectura del scan

    roverbot::Obstacles msg;

    vector<float> normaBot;
    normaBot.clear();

    zBot = cloud.points[0].z; // altura del robot o del laser
    // Caracteristicas del vector referencia al frente del robot
    normaBot.push_back(
	sqrt(
		pow(xBot, 2.0) +
		pow(yBot, 2.0) + 
		pow(zBot, 2.0)
	)
    ); 
    normaBot.push_back(xBot);
    normaBot.push_back(yBot);
    normaBot.push_back(zBot);

    //inicio del vector pasado posicion 0 de la nube de puntos
    vector<float> normaCloudPas; 
    normaCloudPas.clear();
    normaCloudPas.push_back(
	sqrt(
		pow(cloud.points[0].x, 2.0)+
		pow(cloud.points[0].y, 2.0)+
		pow(cloud.points[0].z, 2.0)
	)
    );
    normaCloudPas.push_back(cloud.points[0].x);
    normaCloudPas.push_back(cloud.points[0].y);
    normaCloudPas.push_back(cloud.points[0].z);

    int obstaculos = 1; //número de obstaculos detectados
 
    vector< roverbot::Point > data;
    data.clear();

    //grupo de datos de un objeto
    vector< vector< float > > dataGroup;
    dataGroup.clear();

    for(unsigned int i = 1; i < num_points; i++){

	// Extrae las coordenadas x, y, z de los puntos de la nube
        xCloud = cloud.points[i].x;
        yCloud = cloud.points[i].y;
        zCloud = cloud.points[i].z;

	//Vector de magnitud actual
	vector<float> normaCloud;
        normaCloud.clear();
        normaCloud.push_back(sqrt((xCloud*xCloud)+(yCloud*yCloud)+(zCloud*zCloud)));
	normaCloud.push_back(xCloud);
	normaCloud.push_back(yCloud);
	normaCloud.push_back(zCloud);
	
	float rango = (normaCloud[0] * 10)/100.0; // Tolerancia de error
	
	dataGroup.push_back(normaCloudPas); // añade los datos al grupo

	// si esta dentro de la tolerancia hace parte del grupo de datos del objeto
	// Si no, hace parte del vertice del objeto, por lo tanto entra a la condicion y verifica el punto minimo
	if (normaCloudPas[0] > normaCloud[0]+rango || normaCloudPas[0] < normaCloud[0]-rango){
	    roverbot::Point point;

            //Distancia de comparación
            // ángulo y distancia del punto mas cerca
            vector<float> minDistance = normaBot; 
	    double theta = 0; 
	    // Angulo minimo
            vector<float> minNorma; 
	    double minTheta = 1000;
            //Angulo maximo
            vector<float> maxNorma; 
	    double maxTheta = 0;

	    int sizeGroup = dataGroup.size();
	    vector<float> floatTemp;
            floatTemp.clear();
	    // verifica todos los datos en el grupo encontrando el menor
	    for(unsigned int j = 0; j < sizeGroup; j++){
		
		floatTemp = dataGroup[j];
	        // cos(theta) = InnerPorduct(u, v) / |u|*|v| 
	        norma = normaBot[0]*floatTemp[0]; // |u|*|v| 

	        prodIn = (normaBot[1]*floatTemp[1])+(normaBot[2]*floatTemp[2])+(normaBot[3]*floatTemp[3]); // InnerPorduct(u, v)

	        cosA = prodIn/norma; // InnerPorduct(u, v) / |u|*|v|

	        theta = acos(cosA) * (180 / PI); // theta = cos-1(InnerPorduct(u, v) / |u|*|v| )

	        // punto mas cercano 
	        if(minDistance[0] > floatTemp[0]){
	           minDistance = floatTemp;
	        }

	        // punto minimo en angulo
	        if(minTheta > theta){
	           minTheta = theta;
	           minNorma = floatTemp;
	        }

	        // punto maximo en angulo
	        if(maxTheta < theta){
	           maxTheta = theta;
	           maxNorma = floatTemp;
	        }
			
	    }
	    // Verifica el angulo de donde proviene, si y+ el angulo es positivo, en caso contrario y- el angulo es negativo
	    if(minDistance[2] < 0){
		theta *= -1;
		minTheta *= -1;
		maxTheta *= -1;
	    }
	    // filtra casos donde esta en el borde del radar
	    if(minDistance[0] < xBot-((xBot*1)/100.0)){
		//printf("Obstaculo: %d\n", obstaculos++);
		//printf("Rango de vision sobre el obstaculo:\n"); 
		//printf("angulo --> %4.2f; distancia --> %4.4f m\n", minTheta, minNorma[0]);
		//printf("angulo --> %4.2f; distancia --> %4.4f m\n", maxTheta, maxNorma[0]);
	        //printf("Punto cercano: angulo --> %4.2f; distancia --> %4.4f m\n", theta, minDistance[0]);

		point.min_theta = minTheta;
		point.theta = theta;
		point.max_theta = maxTheta;

		point.min_norm = minNorma[0];
		point.norm = minDistance[0];
		point.max_norm = maxNorma[0];
		
		msg.obstacles.push_back(point);
	    }
	    dataGroup.clear(); // limpia el vector usado
	}
	normaCloudPas = normaCloud; // establece el punto actual como pasado.
    }

    msg.size = msg.obstacles.size();
    msg.label = "ros_topic: roverbot/Obstacles";

    return msg;
	
  }

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_obstacles");
  ros::NodeHandle n;
  while (ros::ok())
  {
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  }
  return 0;
}
