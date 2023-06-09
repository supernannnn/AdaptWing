#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

nav_msgs::Path point()
{
    // Circle parameters
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    double h = 1.0;
    double scale = 0.25;

    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.25;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y =  scale * 0.5;
    pt.pose.position.x =  scale * 5.3;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.75;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 0.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      
    // Return
    return waypoints;
}

// Circle trajectory
nav_msgs::Path circle()
{
    double h = 1.0;
    double scale = 1.0;
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);         

    // Return
    return waypoints;
}

// Figure 8 trajectory
nav_msgs::Path eight()
{
    // Circle parameters
    double offset_x = 0.0;
    double offset_y = 0.0;
    double r = 1.0;
    double h = 1;
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    

    for(int i=0; i< 1; ++i)
    {
        // First loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r ;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x * 2;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);
        // Second loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
    }
    return waypoints;   
}  


nav_msgs::Path self()
{

    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    

    pt.pose.position.x = 0;
    pt.pose.position.y = 0;
    pt.pose.position.z = 1;
    waypoints.poses.push_back(pt);      
    pt.pose.position.x = 1;
    pt.pose.position.y = 1;
    pt.pose.position.z = 1;
    waypoints.poses.push_back(pt);  
    pt.pose.position.x = 1;
    pt.pose.position.y = 2;
    pt.pose.position.z = 3;
    waypoints.poses.push_back(pt);  
    pt.pose.position.x = 4;
    pt.pose.position.y = 4;
    pt.pose.position.z = 5;
    waypoints.poses.push_back(pt);       
    pt.pose.position.x = 5;
    pt.pose.position.y = 6;
    pt.pose.position.z = 5;
    waypoints.poses.push_back(pt);      
    pt.pose.position.x = 6;
    pt.pose.position.y = 7;
    pt.pose.position.z = 7;
    waypoints.poses.push_back(pt);  
    pt.pose.position.x = 9;
    pt.pose.position.y = 9;
    pt.pose.position.z = 10;
    waypoints.poses.push_back(pt);  
    pt.pose.position.x = 10;
    pt.pose.position.y = 11;
    pt.pose.position.z = 12;
    waypoints.poses.push_back(pt);
    return waypoints;   
}  


#endif