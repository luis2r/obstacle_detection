#include "myviz.h"
#include <string>
#include <string.h>
// header kdevelop
#include "../../../ros_independent/include/planner_msgs/flat_walk_plan.h"

void myviz::on_go_there_clicked()
{
    tool_manager_->setCurrentTool(move_tool_);
}

void myviz::on_confirm_path_clicked()
{	
    yarp::os::Bottle temp;
    temp.clear();
    
    temp.addDouble(xn);
    temp.addDouble(yn);
    temp.addDouble(th);
    send_goal.write(temp);
    
    state_manager->setStatus("PLANNING","PROCESSING...");
    ui->confirm_path->setEnabled(false);
}

void myviz::moveCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    tf::Quaternion quat;
    double roll,pitch,yaw;
    
    geometry_msgs::PoseStamped planner_pose;
    tf_.transformPose("/odom", *goal, planner_pose);

    tf::quaternionMsgToTF(planner_pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    
    xn=planner_pose.pose.position.x;
    yn=planner_pose.pose.position.y;
    th=yaw;
    
    QString navgoal ="[ ";
    sx.setNum(xn);
    sy.setNum(yn);
    sz.setNum(th);
    QString sm=", ";
    QString se=" ]";
    navgoal.append(sx).append(sm).append(sy).append(sm).append(sz).append(se);

    ui->plan_list->item(1)->setText(navgoal);
}

void myviz::init_path_marker()
{
    // Path markers
    line_strip.header.frame_id = "odom";
    line_strip.ns = "path";
    line_strip.id=0;
    line_strip_done.header.frame_id = "odom";
    line_strip_done.ns = "path_done";
    line_strip_done.id=1;
    line_strip.type = line_strip_done.type =visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = line_strip_done.scale.x =.1; // line width
    line_strip.color.r = line_strip_done.color.r = .8;
    line_strip.color.g = line_strip_done.color.g =.36;
    line_strip.color.b = line_strip_done.color.b =.36;
    line_strip.color.a = line_strip_done.color.a =1;
    
    waypoints.header.frame_id = "odom";
    waypoints.ns = "waypoints";
    waypoints.id=2;
    waypoints_done.header.frame_id = "odom";
    waypoints_done.ns = "waypoints_done";
    waypoints_done.id=3;
    waypoints.type = waypoints_done.type = visualization_msgs::Marker::LINE_LIST;
    waypoints.scale.x = waypoints_done.scale.x = .1;
    waypoints.color.r = waypoints_done.color.r = .8;
    waypoints.color.g = waypoints_done.color.g =.36;
    waypoints.color.b = waypoints_done.color.b =.36;
    waypoints.color.a = waypoints_done.color.a =1;
    
    // Interactive markers
    /*int_waypoints.header.frame_id = "odom";
    int_waypoints.type =visualization_msgs::Marker::CUBE;
    int_waypoints.scale.x = .1;
    int_waypoints.scale.y = .1;
    int_waypoints.scale.z = .1;
    int_waypoints.pose.orientation.w =1;
    int_waypoints.color.b = 1;
    int_waypoints.color.a = 1;
    */
}
    
    
void myviz::publish_path_marker()
{
      for(int i=0;i<path.size();i++)
      {
      auto p_y=path[i];
      std::cout<<"Path points: "<<p_y.x<<" "<<p_y.y<<" "<<p_y.z<<std::endl<<std::flush;
      p.x = p_y.x;
      p.y = p_y.y;
      p.z = p_y.z;
      /*
      int_waypoints.pose.position.x=p_y.x;
      int_waypoints.pose.position.y=p_y.y;
      int_waypoints.pose.position.z=p_y.z;
      int_waypoints.id=i;
      int_waypoints.ns=i;
      marker_array.markers.push_back(int_waypoints);
      */
      line_strip.points.push_back(p);
      line_strip_done.points.push_back(p);
      waypoints.points.push_back(p);
      waypoints_done.points.push_back(p);
      p.z += 1.0;
      waypoints.points.push_back(p);
      waypoints_done.points.push_back(p);
      }
      path_array.markers.push_back(line_strip);
      path_array.markers.push_back(line_strip_done);
      path_array.markers.push_back(waypoints);
      path_array.markers.push_back(waypoints_done);
      /*marker_array.markers.push_back(line_strip);
      marker_array.markers.push_back(waypoints);*/
      //path_array_pub.publish<visualization_msgs::MarkerArray>(marker_array);
      strip_pub.publish<visualization_msgs::MarkerArray>(path_array);
      
}

void myviz::plan_timer_body()
{

    if (planned_path_g.get_updates(last_planned_path))
    {
     std::cout<<last_planned_path.progress << std::endl;
     
     if (last_planned_path.status==1)
     {
	state_manager->setStatus("PLANNING","NO SOLUTION FOUND");
	ui->confirm_path->setEnabled(true);
     }
     
     if (last_planned_path.status==2)
     {
	state_manager->setStatus("PLANNING","GOAL IN COLLISION");
	ui->confirm_path->setEnabled(true);
     }
     
     if (last_planned_path.status<=0)
     {
     ui->plan_progress_bar->setValue(last_planned_path.progress);
     
     for(flat_walk_cmd cmd:last_planned_path.controls)
     {
	 std::cout << cmd.action << " " << cmd.amount << std::endl;
     }
     
     path_array.markers.clear();

     if(last_planned_path.status==0)
     {
     std::cout << "NEW PLAN RECEIVED!" << std::endl;
     state_manager->setStatus("PLANNING","NEW DATA RECEIVED");
     ui->load_locomotion_command->setEnabled(true);
     ui->go_there->setEnabled(true);
     ui->confirm_path->setEnabled(true);
     }
     
     path.clear();
     line_strip.points.clear();
     waypoints.points.clear();
     line_strip_done.points.clear();
     waypoints_done.points.clear();
     path = last_planned_path.to_path();
     strip_pub.publish<visualization_msgs::MarkerArray>(path_array);
     
     publish_path_marker();
     }
    }
}

void myviz::on_modify_path_clicked()
{
    tool_manager_->setCurrentTool(interact_tool_);
    path_interactive_display->setEnabled(true);
}

void myviz::on_locomotion_start_clicked()
{
    yarp::os::Bottle temp;
    temp.clear();
    temp.add(1);
    locomotion_start.write(temp);
    ui->locomotion_pause->setEnabled(true);
}

void myviz::on_locomotion_pause_clicked()
{
    yarp::os::Bottle temp;
    temp.clear();
    temp.add(1);
    locomotion_pause.write(temp);
    ui->locomotion_pause->setEnabled(true);
}

void myviz::on_locomotion_stop_clicked()
{
    yarp::os::Bottle temp;
    temp.clear();
    temp.add(1);
    locomotion_stop.write(temp);
    ui->locomotion_stop->setEnabled(true);
}

void myviz::on_stop_planning_button_clicked()
{
   //TODO: STOP PLANNING
}

void myviz::path_i_m_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{	

   for (int i=0;i<marker_array.markers.size();i++)
   {
   visualization_msgs::Marker temp;    
   temp=marker_array.markers.at(i);
   temp.pose=feedback.pose;
   marker_array.markers.at(i)=temp;
   }
}

void myviz::on_clear_path_clicked()
{
    tool_manager_->setCurrentTool(move_camera_tool_);    
    path_interactive_display->setEnabled(false);
}

void myviz::on_load_locomotion_command_clicked()
{
    std::ostringstream size_;
    std::string nr_;
    std::string size = "Nr of controls: ";
    std::string nr = "Control nr: ";
    std::string type = "Control type: ";
    std::string value = "Control value: ";
    
    size_<<last_planned_path.controls.size();
    size.append(size_.str());
    ui->locomotion_list->item(0)->setText(QString::fromStdString(size));
    
    if (k==0)
    {
    flat_walk_cmd cmd=last_planned_path.controls.at(k);
    std::ostringstream temp;
    temp << k+1;
    nr.append(temp.str());
    type.append(cmd.action);
    temp.clear();
    temp << cmd.amount;
    value.append(temp.str());
    
    ui->locomotion_list->item(1)->setText(QString::fromStdString(nr));
    ui->locomotion_list->item(2)->setText(QString::fromStdString(type));
    ui->locomotion_list->item(3)->setText(QString::fromStdString(value));
    ui->send_locomotion_command->setEnabled(true);
    }
    else
    {
      ui->send_locomotion_command->setEnabled(false);
    }    
}

void myviz::on_send_locomotion_command_clicked()
{	
    if (k>last_planned_path.controls.size()-1)
    {
      ui->send_locomotion_command->setEnabled(false);
    }
    else
    {
    flat_walk_cmd cmd=last_planned_path.controls.at(k);
    yarp::os::Bottle command;
    command.clear();
    command.addString(cmd.action);
    command.addDouble(cmd.amount);
    command.add(k+1);
    locomotion_send.write(command);
    }
}

void myviz::locomotion_timer_body()
{	
    std::ostringstream size_;
    std::string nr_;
    std::string size = "Nr of controls: ";
    std::string nr = "Control nr: ";
    std::string type = "Control type: ";
    std::string value = "Control value: ";
    std::string state;
    int cmd_id;
    
    yarp::os::Bottle* temp = locomotion_status.read();
    
    state = temp->get(0).asString();
    cmd_id = temp->get(1).asInt();
    
    if (temp==NULL)
    {
      state_manager->setStatus("LOCOMOTION","NO DATA RECEIVED");
    }
    if (state=="ready")
    {
      state_manager->setStatus("LOCOMOTION","READY");
    }
    if (state=="walking")
    {
      state_manager->setStatus("LOCOMOTION","WALKING");
    }
    
   if(cmd_id==k+1)
   {
    if (k>last_planned_path.controls.size()-1)
    {
      ui->locomotion_list->item(1)->setText(QString("end of data"));
      ui->locomotion_list->item(2)->setText(QString("end of data"));
      ui->locomotion_list->item(3)->setText(QString("end of data"));
      ui->load_locomotion_command->setEnabled(false);
    }
    else if (k>k+1 && k<=last_planned_path.controls.size()-1)
    {
    flat_walk_cmd cmd=last_planned_path.controls.at(k);
    std::ostringstream temp;
    temp << k+1;
    nr.append(temp.str());
    type.append(cmd.action);
    temp.clear();
    temp << cmd.amount;
    value.append(temp.str());
    // change list tab
    ui->locomotion_list->item(1)->setText(QString::fromStdString(nr));
    ui->locomotion_list->item(2)->setText(QString::fromStdString(type));
    ui->locomotion_list->item(3)->setText(QString::fromStdString(value));
    
    // change markers color
    
    publish_path_marker();
    k++;
    }
   }
   
}


