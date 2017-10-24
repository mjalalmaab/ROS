//  ///////////////////////////////////////////////////////////
//
// turtlebot_mapping_localization.cpp
// 
// Author :  Mehdi Jalalmaab
// Version: 2.1 ; 27 November 2014
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>    // std::min
#define MEAN 0
#define STDDEV 1.0

using namespace std;

nav_msgs::OccupancyGrid    mappp;

ros::Publisher marker_pub;
int selection=0;
#define TAGID 0



	std::default_random_engine generator_uniform;
	std::uniform_real_distribution<double> uniform_distribution(0,1);
	
	std::default_random_engine generator_uniform_dir;
	std::uniform_real_distribution<double> uniform_distribution_dir(-3,3);
	
	visualization_msgs::Marker liness;
	
double X_indoor;
double Y_indoor;
double Yaw_indoor;
double resolution_map;
int X_grid_n;
int Y_grid_n; 
double map_origin_x;
double map_origin_y;
double map_origin_yaw;
int map_data_length; 

int total_needed_milestones;


   // feedback control additions
   double vg =0;
   double wg = 0;
   double x_desired = 0;
   double y_desired = 0;
   double theta_desired = 0;
   double x_diff = 0;
   double y_diff = 0;
   double theta_diff = 0;
   double distance_error = 0;
   double error_derivative = 0;
   double previous_error = 0;
   int counter4=0;
	
	int simulation_time=0;
	int counter = 0;
	int number_of_milestones = 1;



geometry_msgs::Point ppp;


//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	X_indoor = msg.pose.pose.position.x; // Robot X psotition
	Y_indoor = msg.pose.pose.position.y; // Robot Y psotition
 	Yaw_indoor = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	//std::cout << "X: " << X_indoor << ", Y: " << Y_indoor << ", Yaw: " << Yaw_indoor << std::endl ;
}

//Example of drawing a curve



//max begin these 2 functions find the maximum and the index
double Max(double* Numbers,int Count)
{
	double Maximum = Numbers[0];
	for(int i = 0; i < Count; i++)
	{ 
	if( Maximum < Numbers[i] )
	Maximum = Numbers[i];
	}
	return Maximum;
}



int Max2(double* Numbers,int Count)
{
	double Maximum = Numbers[0];
	int indexx = 0;
	for(int i = 0; i < Count; i++)
	{ 
	if( Maximum < Numbers[i] )
	indexx = i;
	Maximum = Numbers[i];
	}
	return indexx;
}
//max end



//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{

	mappp = msg;

	resolution_map   = mappp.info.resolution;
	X_grid_n         = mappp.info.width;       
	Y_grid_n         = mappp.info.height;    
	map_origin_x     = mappp.info.origin.position.x;
	map_origin_y     = mappp.info.origin.position.y;
	map_origin_yaw   = tf::getYaw(mappp.info.origin.orientation);
	map_data_length  = X_grid_n * Y_grid_n; 
	
	
//you probably want to save the map into a form which is easy to work with
}

int input_counter = 0;





int marker_id = 1;


   
		


int main(int argc, char **argv)
{

	//Initialize the ROS framework
    	ros::init(argc,argv,"main_control");
    	ros::NodeHandle n;


    	//Subscribe to the desired topics and assign callbacks
    	ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    	ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    	//Setup topics to Publish from this node
    	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    	//Velocity control variable
    	geometry_msgs::Twist vel;




   
   

	//ros::spinOnce();   //Check for new messages
	
	//cout<<"\n \n \n mappp.info.resolution \n \n \n";
	//cout<<mappp.info.resolution;

    	//This function is called when a new map is received
	


	double initial_state[3];
	double goal[3];
	
		//goal[0] = 4;
		//goal[1] = 0;
		//goal[2] = 0;

                //goal[0] = 8;
		//goal[1] = -4;
		//goal[2] = 0;
		
		goal[0] = 8;
		goal[1] = 0;
		goal[2] = 0;


	double current_state[3];
	double next_state[3];
	int total_try = 1000;
	double milestones[4][total_try];
	double connection_map[7][total_try];
	int chosen_milestone;
	double new_milestone[3];
	double max3;
	int found_new_milestone = 0;
	double time_span;

	double control_input[2];
	double new_state[3];
	 
	// BEGIN RRT
	// Initialize start point as first milestone
	

	double new_map[5][total_try];



int list_of_milestones[total_try];
double parent_milestone;
int last_milestone;


int n_milestone = 1;



	     marker_id= marker_id +1;


   liness.header.frame_id = "/map";
   liness.id = marker_id ; //each curve must have a unique id or you will overwrite an old ones
   liness.type = visualization_msgs::Marker::LINE_STRIP;
   liness.action = visualization_msgs::Marker::ADD;
   liness.ns = "curves";
   liness.scale.x = 0.1;
   liness.color.r = 1.0;
   liness.color.b = 0.2*marker_id;
   liness.color.a = 1.0;




	
 int path_generation =1;
	cout<<"\n \n path_generation out of while ros ok \n \n";



    	//Set the loop rate
    	ros::Rate loop_rate(5);    //20Hz update rate
	

 	while (ros::ok())
	{

    		loop_rate.sleep(); //Maintain the loop rate
    		ros::spinOnce();   //Check for new messages
		cout<<"path_generation inside of while ros ok";
		
		int initial_pos_active = 1;
		if (initial_pos_active ==1)
		{
		initial_pos_active = 0;
		initial_state[0] = X_indoor;
		initial_state[1] = Y_indoor;
		initial_state[2] = Yaw_indoor;
		
		
		x_desired = X_indoor;
		y_desired = Y_indoor;
   		theta_desired = Yaw_indoor;
   
   
   

   
   
   
		//cout<<" \n initial x \n" ;
		//cout<< initial_state[0];
		
		//cout<<" \n initial_y \n ";
		//cout<<initial_state[1];
		
		//cout<<" \n initial_yaw \n ";
		//cout<<initial_state[2];
		//cin.get();
		
		current_state[0] = initial_state[0];   //0000000000000
		current_state[1] = initial_state[1];   //0000000000000
		current_state[2] = initial_state[2];   //0000000000000
		
		milestones[0][0] = 1;
		milestones[1][0] = current_state[0];
		milestones[2][0] = current_state[1];
		milestones[3][0] = current_state[2];
	 	}
	 	
	 	//Draw Curves
         	//drawCurve(1);
         	//drawCurve(2);
         	//drawCurve(4);
    
    		//Main loop code goes here:

		////////////////////////////
		//////////////////////////// 
		// initialize everything
		////////////////////////////
		////////////////////////////


		//cout<<"\nresolution_map\n";
		//cout<<mappp.info.resolution;
		//cout<<"\nY_grid_n\n";
		//cout<<Y_grid_n;


		//cout<<"\nX_grid_n\n";
		//cout<<X_grid_n;

		//cout<<"\nmap_data_length\n";
		//cout<<map_data_length;















		while (path_generation ==1)
		{
		path_generation =0;
		// Initialization for Path Planning
		double X_cells[X_grid_n];    
		double Y_cells[Y_grid_n];
	

		cout<<" \n \n \n X_grid_n\n \n \n";
		cout<< X_grid_n;
		for(int a=0; a<X_grid_n; a++)
		{
			X_cells[a] =  map_origin_x + resolution_map * a + resolution_map / 2;
			cout<<"   X_cells[a]: "  ;
			cout<<X_cells[a];
		}
	

		for(int a=0; a<Y_grid_n; a++)
		{
			Y_cells[a] =  map_origin_y + resolution_map * a + resolution_map / 2;
			cout<<"   y_cells[a]: "  ;
			cout<<Y_cells[a];
		}
		
		cout<<" \n X_grid_n: "  ;
		cout<<     X_grid_n   ;

		cout<<" \n Y_grid_n: "  ;
		cout<<     Y_grid_n     ;
		
		cout<<" \n map_origin_x: ";
		cout<<     map_origin_x  ;
		
		cout<<" \n map_origin_y: ";
		cout<<     map_origin_y;
		
		cout<<" \n map_origin_yaw: ";
		cout<<     map_origin_yaw;
		
		cout<<" \n map_data_length: ";
		cout<<     map_data_length;
		
		
     
		double map_matrix[X_grid_n][Y_grid_n];

		for(int a=0; a<X_grid_n; a++)
		{
			for(int b=0; b<Y_grid_n; b++)
			{
			map_matrix[b][a] = mappp.data[a*X_grid_n+b];
			
			//cout<<" \n map_matrix[a][b]: " << a << " , " << b << " \n ";
			//cout<<     map_matrix[a][b];
			}
     		}
	




		// MAIN LOOP
		while (number_of_milestones < total_try)
		{
			double weights[number_of_milestones];
			double distance_to_goal[number_of_milestones];
			double random_weights[number_of_milestones];
			
			
			
			
			//simulation_time ++;
			// select which milestone to expand
		
			//cout<<" \n number_of_milestones: ";
			//cout<<     number_of_milestones;
		
			for(int a=0; a<number_of_milestones; a++)
			{
			//cout<<" \n milestone: " << a << " pos" <<  milestones[1][a] << " $ " << milestones[2][a] ;
			double xdifference = milestones[1][a]-goal[0];
			double ydifference = milestones[2][a]-goal[1];
	    		
	    		
	    		//cout<<" \n xdifference: ";
			//cout<<     xdifference;
	    		
	    		
	    		//cout<<" \n ydifference: ";
			//cout<<     ydifference;
			
			
			distance_to_goal[a] = sqrt(pow(xdifference,2)+pow(ydifference,2));
	    
			weights[a] = exp(-distance_to_goal[a]);
			
			//cout<<" \n distance_to_goal: ";
			//cout<<     distance_to_goal;
			
			//cout<<" \n weights[a]: ";
			//cout<<     weights[a];
			
			
			}
			
			

			// now sample which milestone to expand based on weights and some randomness
		    
			for (int a_ms=0; a_ms<number_of_milestones; a_ms++)
			{
			//cout<<" \n number_of_milestones \n ";
			//cout<<number_of_milestones;
			random_weights[a_ms] = weights[a_ms] * uniform_distribution(generator_uniform);
			
			//cout<<" \n random_weights[a]: ";
			//cout<<     random_weights[a_ms];
			
			}
			chosen_milestone = Max2(random_weights , number_of_milestones);
			
			cout<<" \n distance_to_goal \n";
	    		cout<<distance_to_goal[chosen_milestone];
	    		
	    		
	    		//cout<<" \n number_of_milestones: " << number_of_milestones  <<" chosen_milestone: " << chosen_milestone;
			//cout<<     chosen_milestone;
			
			

			current_state[0] = milestones[1][chosen_milestone];
			current_state[1] = milestones[2][chosen_milestone];
			current_state[2] = milestones[3][chosen_milestone];
			
			
			for(int n_try_new_milestone=0; n_try_new_milestone<100; n_try_new_milestone++)
			{
	    			//cout<<" \n n_try_new_milestone \n";
	    			//cout<<n_try_new_milestone;
	    			
	    			time_span = 0.5; //uniform_distribution(generator_uniform)/2;

				control_input[0] = uniform_distribution(generator_uniform);
				control_input[1] = uniform_distribution_dir(generator_uniform_dir);
	        		
	    			
	    			//control_input[0] = uniform_distribution(generator_uniform);
				//control_input[1] = uniform_distribution_dir(generator_uniform_dir);
	    
	    			//cout<<" \n time_span: ";
				//cout<<     time_span;
				
				//cout<<" \n control_input[0]: ";
				//cout<<     control_input[0];
				
				//cout<<" \n control_input[1]: ";
				//cout<<     control_input[1];
				
				
				// using the random control input and the motion model propogate the robot position
		
				double t_step = 0.01; 
				//cin.get();
				for (double a=0; a<time_span; a= a + t_step)
				{
					new_state[0] = current_state[0] + control_input[0] * cos(current_state[2]) * t_step;   // dt = 1
					new_state[1] = current_state[1] + control_input[0] * sin(current_state[2]) * t_step;   // dt = 1
					new_state[2] = current_state[2] + control_input[1] * t_step;
	    	
	    				//cout<<" \n new_state[0]: ";
					//cout<<     new_state[0];
					
					//cout<<" \n new_state[1]: ";
					//cout<<     new_state[1];
					
					//cout<<" \n new_state[2]: ";
					//cout<<     new_state[2];
					
					//cout<<" \n time";
					//cout<< a;
					//

					
   
					double Cell_dis[X_grid_n][Y_grid_n];
					for(int i = 0; i < X_grid_n; i++)
					{ 
						for(int j = 0; j < Y_grid_n; j++)
						{
						Cell_dis[i][j] = sqrt(pow(new_state[0]-X_cells[i],2)+pow(new_state[1]-Y_cells[j],2));
						}
					}
	

	
					double min_dis = Cell_dis[0][0];
					int k_min_i  = 0;
					int k_min_j  = 0;
					double min_x;
					double min_y;
					for(int i = 0; i < X_grid_n; i++)
					{ 
						for(int j = 0; j < Y_grid_n; j++)
						{
							if( min_dis >  Cell_dis[i][j] )
							{
							min_dis   = Cell_dis[i][j];
							k_min_i = i;
							k_min_j = j;
							min_x   = new_state[0]-X_cells[i];
							min_y   = new_state[1]-Y_cells[j];
							}
						}
					}
					
					
	
					//cout<<" \n min_x: ";
					//cout<<     min_x;
					
					//cout<<" \n k_min_x: ";
					//cout<<     k_min_x;
					
					//cout<<" \n min_y: ";
					//cout<<     min_y;
					
					//cout<<" \n k_min_y: ";
					//cout<<     k_min_y;
					
					//cin.get();					


					//check at each time step if we hit an obstacle
					if ((map_matrix[k_min_i][k_min_j] > 50) && (min_dis <= 20 ) ) // && (min_y <= 1) ) 
					// get x and y obstacle positions from the map and compare them to the robot position at this time step
					{
		        			//cout<<" \n collision \n ";
		        			break; // break for loop
		       			}
	        


		        		if( a>=time_span - t_step ) // if we made it through all the time steps with no collision
		        		{
		                		n_try_new_milestone = 1000; // set collisions to '1' to exit the while loop
		                		new_milestone[0] = new_state[0];
						new_milestone[1] = new_state[1];
						new_milestone[2] = new_state[2];
						//cout<<" \n YoHooooooooo  New Milestone   \n ";
						
	    	
						milestones[0][number_of_milestones] = number_of_milestones+1;
						milestones[1][number_of_milestones] = new_milestone[0];
						milestones[2][number_of_milestones] = new_milestone[1];
						milestones[3][number_of_milestones] = new_milestone[2];
						
						
	    
						// connection_map is an array that tracks how each milestone is connected to its previous (parent) milestone. 
						//It also tracks the control input needed to go from the parent milestone to the 	next milestone
						connection_map[0][number_of_milestones] = number_of_milestones+1;
						connection_map[1][number_of_milestones] = chosen_milestone;
						connection_map[2][number_of_milestones] = control_input[0];
						connection_map[3][number_of_milestones] = control_input[1];
						connection_map[4][number_of_milestones] = time_span;
						connection_map[5][number_of_milestones] = new_milestone[0];
						connection_map[6][number_of_milestones] = new_milestone[1];
						connection_map[7][number_of_milestones] = new_milestone[2];
						number_of_milestones = number_of_milestones +1;
						//cin.get();
						
        	    			}
		    
					current_state[0] = new_state[0]; // propogate the motion model by one step
					current_state[1] = new_state[1];
					current_state[2] = new_state[2];

                                        //cin.get();

					//cout<<"\n \n \n \n current_state[0]\n";
					//cout<<current_state[0];
					//cout<<"\ncurrent_state[1]\n";
					//cout<<current_state[1];
					//cout<<"\ncurrent_state[2]\n";
					//cout<<current_state[2];


				}     // end for loop

			}  
			//break  ;         // end for new milestone loop

	            	//cout<<" \n Yohoo Again";
			//cin.get();
			// add new_milestone to list of all milestones
			
	    
			// check to see if our newest milestone is at - or close to - the goal
			if (( abs(new_state[0] - goal[0])<0.5 ) && (abs(new_state[1] - goal[1])<0.5)) 
			{
			cout<<" \n goaaaaaaaaaaaaaaaaaaaaaaaaal \n";	
			break; // break the while loop. We have a path from start to finish

			}
			
		//cin.get();
		//break;
		} // end number of mile stones
		//cout<<"simulation_time  \n ";
		//cout<<simulation_time;

	//break;
	/*
	
		for(int i_number_of_milestones = 0; i < number_of_milestones; i++)
		{
		milestone_series[i] = connection_map[1][number_of_milestones-i];
		
		
		} 
	*/
	// now recursively reconstruct the path backward from the goal to the start
	
	// construct a list of milestones to go to

	last_milestone = number_of_milestones;
	

	
	
	
	
	list_of_milestones[total_try-1]=number_of_milestones; // our last milestone is our goal
	
	int counter2 = 0;
	parent_milestone = last_milestone;
	
	while(parent_milestone !=0)
	{
 		cout<<" \n last_milestone \n";
		cout<<last_milestone;
		parent_milestone = connection_map[1][last_milestone-1];   
 		
 		
 		cout<<" \n connection_map[1][last_milestone] \n";
	        cout<<connection_map[1][last_milestone-1];
	        
	        
	
	    	list_of_milestones[(total_try-counter2-1)] = parent_milestone;
	    	
	    	cout<<" \n list_of_milestones \n";
	        cout<<list_of_milestones[(total_try-counter2-1)];
	    	
	
	
			
		cout<<" \n (total_try-counter2-1) \n ";
		cout<<(total_try-counter2-1);
		
		geometry_msgs::Point p;
       		p.x = connection_map[5][last_milestone-1];
      		p.y = connection_map[6][last_milestone-1];
      		p.z = 0; //not used
       		liness.points.push_back(p); 
		marker_pub.publish(liness);
		
		
		
		
    		last_milestone = parent_milestone;
	    
	    
	    
	    
	    
	    
    		counter2 = counter2+1;
	}

	total_needed_milestones = counter2; // the number of milestones we go through
	
	cout<<" \n total_needed_milestones \n";
	cout<<total_needed_milestones;
/*
	for (int a=total_needed_milestones; a>0; a--)
	{
		new_map[0][a] = connection_map[0][list_of_milestones[(total_try-a)]];
		new_map[1][a] = connection_map[2][list_of_milestones[(total_try-a)]];
		new_map[2][a] = connection_map[3][list_of_milestones[(total_try-a)]];
		new_map[3][a] = connection_map[4][list_of_milestones[(total_try-a)]];
		cout<< " \n milestone : " << a << " = " << new_map[0][a];
	} 
	
	new_map[4][0] = 0;  //initial time
	for (int a= 1; a<total_needed_milestones; a++)
	{
		new_map[4][a] = new_map[3][a] + new_map[4][a-1];
	}


*/




	
	
	
	
	} //end og while path generation
	
	
	
	
	




	//break;
	cout<<"\n \n \npath_generation bottom of while path generation \n \n \n ";
	cout<<path_generation;


	
	
	//cout<<"connection_map[0][number_of_milestones]";
	//cout<<connection_map[0][number_of_milestones];
	
	
	
	

	
    		
    		
    		
    		
    		
    		
    		
    		
	
	
	
	
	
	//ros::Rate loop_rate(5);    //20Hz update rate

	//vel.linear.x  = new_map[1][input_counter];
	//vel.angular.z = new_map[2][input_counter];
	
	//velocity_publisher.publish(vel); // Publish the command velocity
	
	 //if we are close to the next milestone x and y coordinates we switch to the next input vector
	 
	n_milestone = int(list_of_milestones[(total_try-total_needed_milestones+input_counter+1)]);
	
	cout<<" \n (total_try-total_needed_milestones+input_counter-1 \n ";
	cout<<(total_try-total_needed_milestones+input_counter+1);
		
	cout<<" \n n_milestone \n";
	cout<<n_milestone;
	
	if (   (abs( X_indoor - connection_map[5][n_milestone]    )<1 ) && ( (abs( Y_indoor - connection_map[6][n_milestone]    )<1)  ) )
	{
 	input_counter = input_counter + 1; // this tracks which set of randomly generated inputs we are currently applying
 	
 	n_milestone = int(list_of_milestones[(total_try-total_needed_milestones+input_counter+1)]);
	
	cout<<" \n (total_try-total_needed_milestones+input_counter-1 \n ";
	cout<<(total_try-total_needed_milestones+input_counter+1);
		
	cout<<" \n n_milestone \n";
	cout<<n_milestone;
	}
	
	//if (distance_to_goal<0.26)
	//{
	//    break; // break main loop. we are at our destination
	//}



	

	//if (   abs(vel.linear.x) + abs(vel.linear.z) < 0.1 )
	//{
 	//input_counter = input_counter + 1; // this tracks which set of randomly generated inputs we are currently applying
 	
 	//n_milestone = int(list_of_milestones[(total_try-total_needed_milestones+input_counter+1)]);
	
	//cout<<" \n (total_try-total_needed_milestones+input_counter-1 \n ";
	//cout<<(total_try-total_needed_milestones+input_counter+1);
		
	//cout<<" \n n_milestone \n";
	//cout<<n_milestone;
	//}







		
		//Main loop code goes here:
    		//vel.linear.x = 0.1; // set linear speed
    		//vel.angular.z = 0.3; // set angular speed
    		//velocity_publisher.publish(vel); // Publish the command velocity

                 


		x_desired = connection_map[5][n_milestone] ;   // dt = 1
		y_desired = connection_map[6][n_milestone];   // dt = 1
		theta_desired = connection_map[7][n_milestone];


		//x_desired = x_desired + connection_map[1][n_milestone] * cos(theta_desired);   // dt = 1
		//y_desired = y_desired + connection_map[1][n_milestone] * sin(theta_desired);   // dt = 1
		//theta_desired = theta_desired + connection_map[2][n_milestone];

		x_diff = x_desired - X_indoor;
		y_diff = y_desired -Y_indoor;
		theta_diff = theta_desired - Yaw_indoor;


		wg = atan2(y_diff,x_diff);
		
		double theta_error =  wg - Yaw_indoor;
		
		if (theta_error > 2 * M_PI)
		theta_error = theta_error - 2 * M_PI;
		
		if (theta_error < -M_PI/2)
		theta_error = theta_error + 2 * M_PI;
		
		
		double rot_com = 0.5*(-theta_error);
		
		if (rot_com > 0.5)
		rot_com = 0.5;
		
		if (rot_com < -.5)
		rot_com = -0.5;
		
		
		
		
		//if(theta_diff>(.1) || (theta_diff<-.1))
		//{
		//wg=-theta_diff*0.05;
		//}
		//else 
		//{
		//wg = 0;
		//distance_error = sqrt(pow(x_diff,2)+pow(y_diff,2));
		//error_derivative = distance_error;
		//previous_error = distance_error;

		//vg = 1.5*(error_derivative);
		
		//if (vg >1)
		//vg = 1;
		
		//if (vg<-1)
		//vg = -1;
		//}

		cout<< " \n x des   " <<  x_desired << "  X_indoor  "<< X_indoor << "\n";
		cout<< " \n y des   " <<  y_desired << "  Y_indoor  "<< Y_indoor << "\n";
		cout<< " \n wg des   " <<  wg << "  Yaw_indoor  "<< Yaw_indoor  << "  theta_error  " << theta_error << "resolution_map"<< resolution_map<< "\n";
		
		
		
		cout<< " \n vel v " << 0.1 << "\n";
		cout<< " \n vel w " << rot_com << "\n";
		
		if (abs(rot_com)>0.1)
		{
		vel.linear.x  = 0;               //connection_map[2][n_milestone] ;//+ vg;
		vel.angular.z = abs(rot_com);   //connection_map[3][n_milestone] ;//+ wg;
		}
		else
		{
		vel.linear.x  = .1;               //connection_map[2][n_milestone] ;//+ vg;
		vel.angular.z = rot_com;   //connection_map[3][n_milestone] ;//+ wg;
		}
		//vel.linear.x  = .1;               //connection_map[2][n_milestone] ;//+ vg;
		//vel.angular.z = rot_com;   //connection_map[3][n_milestone] ;//+ wg;
		
		cout<< " \n vel v " << vel.linear.x << "\n";
		cout<< " \n vel w " << vel.angular.z << "\n";
		
		
		velocity_publisher.publish(vel); // Publish the command velocity
		cout<<"yesssssssssssssss";




		
	 
	}
	   return 0;
}
