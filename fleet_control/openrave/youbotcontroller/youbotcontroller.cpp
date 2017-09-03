#include <openrave/openrave.h>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include "brics_actuator/JointValue.h"
#include "brics_actuator/JointPositions.h"

#include "assembly_common/BasePose.h"
#include "assembly_common/BaseCommand.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <boost/algorithm/string.hpp>


using namespace std;
using namespace OpenRAVE;

class YoubotController : public ControllerBase
{
public:
    YoubotController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv) //, cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false)
    {
        __description = ":Interface Author: Dalitso Banda";

        RegisterCommand("MoveArm",boost::bind(&YoubotController::MoveArm, this,_1,_2),"moves the arm ");
        RegisterCommand("MoveGripper",boost::bind(&YoubotController::MoveGripper, this,_1,_2),"moves the gripper ");
        RegisterCommand("MoveBase",boost::bind(&YoubotController::MoveBase, this,_1,_2),"moves the gripper ");

        //RegisterCommand("SetThrowExceptions",boost::bind(&YoubotController::_SetThrowExceptions,this,_1,_2),
        //                "If set, will throw exceptions instead of print warnings. Format is:\n\n  [0/1]");

        _nControlTransformation = 0;
        _penv = penv;
    }

    bool MoveArm(ostream& sout, istream& sinput)
    {
        ros::Rate loop_rate(10);

        _joint_angles_pub = _pn->advertise<brics_actuator::JointPositions>((string(_probot->GetName())+"/arm_1/arm_controller/position_command").c_str(), 1);
        std::string valuesString;
        sinput >> valuesString;

        while(ros::ok()){
            ROS_INFO("input %s", valuesString.c_str());

            vector<std::string> valuestokens;
            boost::split(valuestokens, valuesString, boost::is_any_of("\t,"));

            vector<double> values;

            for(size_t i = 0; i < valuestokens.size(); ++i)
            {
                values.push_back(boost::lexical_cast<double>(valuestokens[i]));
            }

            //vector<string> jointNametokens;
            //boost::split(jointNametokens, values, boost::is_any_of("\t "));

            brics_actuator::JointPositions command;
            vector <brics_actuator::JointValue> armJointPositions;
            vector <std::string> armJointNames;
            armJointPositions.resize(7);
            armJointNames.resize(7);

            armJointNames[0]="joint 1";
            armJointNames[1]="joint 2";
            armJointNames[2]="joint 3";
            armJointNames[3]="joint 4";
            armJointNames[4]="joint 5";
            armJointNames[5]="joint 6";
            armJointNames[6]="joint 7";

            for(unsigned int i = 0; i < 7; ++i)
            {
                armJointPositions[i].joint_uri = armJointNames[i].c_str();
                armJointPositions[i].value = values[i];
                armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
            }

            command.positions = armJointPositions;
            ROS_INFO("the values are %f %f %f %f %f %f %f", values[0], values[1],values[2], values[3],values[4], values[5], values[6]);
            _joint_angles_pub.publish(command);
            ros::spinOnce();
            loop_rate.sleep();
            return true;
        }


    }

    bool MoveGripper(ostream& sout, istream& sinput)
    {
        ros::Rate loop_rate(10);

        _joint_angles_pub = _pn->advertise<brics_actuator::JointPositions>((string(_probot->GetName())+"/gripper_controller/position_command").c_str(), 1);
        std::string valuesString;
        sinput >> valuesString;
        while(ros::ok()){
            ROS_INFO("input %s", valuesString.c_str());

            vector<std::string> valuestokens;
            boost::split(valuestokens, valuesString, boost::is_any_of("\t,"));

            vector<double> values;
            for(size_t i = 0; i < valuestokens.size(); ++i)
            {
                values.push_back(boost::lexical_cast<double>(valuestokens[i]));
            }


            brics_actuator::JointPositions command;
            vector <brics_actuator::JointValue> gripperJointPositions;
            vector <std::string> gripperJointNames;
            gripperJointPositions.resize(7);
            gripperJointNames.resize(7);

            gripperJointNames[0]="gripper 1";
            gripperJointNames[1]="gripper 2";
            gripperJointNames[2]="gripper 3";
            gripperJointNames[3]="gripper 4";
            gripperJointNames[4]="gripper 5";
            gripperJointNames[5]="gripper 6";
            gripperJointNames[6]="gripper 7";

            for(unsigned int i = 0; i < 7; ++i)
            {
                gripperJointPositions[i].joint_uri = gripperJointNames[i].c_str();
                gripperJointPositions[i].value = values[i];
                gripperJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
            }

            command.positions = gripperJointPositions;
            ROS_INFO("the values are %f %f %f %f %f %f %f", values[0], values[1],values[2], values[3],values[4], values[5], values[6]);
            _joint_angles_pub.publish(command);
            ros::spinOnce();
            loop_rate.sleep();
            return true;
        }


    }

    bool move_base(assembly_common::BasePose::Request  &req,
             assembly_common::BasePose::Response &res)
    {
        _joint_angles_pub = _pn->advertise<assembly_common::BaseCommand>((string(_probot->GetName())+"/robot_base_command").c_str(), 1);

        assembly_common::BaseCommand command;
        command.goal_x =  req.x;
        command.goal_y = req.y;
        command.goal_theta = req.theta;
        res.complete  = true;
        ROS_INFO("request: x=%f, y=%f", req.x, req.y);
        ROS_INFO("sending back response: [%d]", res.complete);

        _joint_angles_pub.publish(command);
        return true;
    }

    bool MoveBase(ostream& sout, istream& sinput)
    {
        assembly_common::BasePose srv;

        std::string valuesString;
        sinput >> valuesString;

        ROS_INFO("input %s", valuesString.c_str());

        vector<std::string> valuestokens;
        boost::split(valuestokens, valuesString, boost::is_any_of("\t,"));

        srv.request.x = boost::lexical_cast<double>(valuestokens[0]);
        srv.request.y = boost::lexical_cast<double>(valuestokens[1]);
        srv.request.theta = boost::lexical_cast<double>(valuestokens[2]);
        srv.request.threshold = boost::lexical_cast<double>(valuestokens[3]);
        srv.request.angle_threshold = boost::lexical_cast<double>(valuestokens[4]);
        srv.request.arm_offset = std::strcmp(valuestokens[5].c_str(),"True");
        srv.request.frame = valuestokens[6];
        return client.call(srv);
    }
    virtual ~YoubotController()
    {
    }

    void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if (msg->name.at(0) != string("arm_joint_1"))   // wheel joints are published to the same topic. Ignore them.
        {
            return;
        }

        std::vector <double> results; // this will store our final dof values

        // read the values form the ros topic
        std::vector<double> readV = msg->position;

        // subtract the offset from the read values
        for(unsigned int i =0; i < readV.size(); i++)
        {
            results.push_back(readV.at(i) - _offset.at(i));
        }

        _probot->SetDOFValues(results,0); // The 0 is to not check limits.
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;

        if( !!_probot )
        {
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;
        }
        // intialize ros and the ros node handle
        //int argc = 0;
        //char** argv = NULL;
        //ros::init(argc, argv, "youbotcontroller");
        _pn = new ros::NodeHandle();

        // create ros subscriber for joint messages.
        _joint_angles_sub = _pn->subscribe((string(_probot->GetName())+"/joint_states").c_str(), 1, &YoubotController::JointStateCallback, this);

        // start the move base service
        service = _pn->advertiseService("move_base", &YoubotController::move_base, this);
        client = _pn->serviceClient<assembly_common::BasePose>("move_base");

        _offset.clear();
        double offsetvals[] = {2.950, 1.1345, -2.5482, 1.7890, 2.9234, 0.0, 0.0 ,0.0};
        for (int i =0 ; i < 8; i++)
        {
            _offset.push_back(offsetvals[i]);
        }

        return true;
    }

    virtual void Reset(int options)
    {
    }

    virtual const std::vector<int>& GetControlDOFIndices() const
    {
        return _dofindices;
    }

    virtual int IsControlTransformation() const
    {
        return _nControlTransformation;
    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
        ros::spinOnce();
    }

    virtual bool IsDone()
    {
        return true;
    }

    virtual dReal GetTime() const
    {
        return 0;
    }
    virtual RobotBasePtr GetRobot() const
    {
        return _probot;
    }

private:

    RobotBasePtr _probot;               ///< controlled body
    std::vector<int> _dofindices;
    int _nControlTransformation;
    ros::Subscriber _joint_angles_sub;
    ros::Publisher _joint_angles_pub;
    ros::NodeHandle* _pn;
    ros::ServiceServer service;
    ros::ServiceClient client;
    EnvironmentBasePtr _penv;
    std::vector<double> _offset;

};

ControllerBasePtr CreateYoubotController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new YoubotController(penv,sinput));
}

