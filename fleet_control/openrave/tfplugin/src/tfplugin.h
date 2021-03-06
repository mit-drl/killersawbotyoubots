#ifndef TFPLUGIN_H_
#define TFPLUGIN_H_

#include <map>
#include <set>
#include <openrave/openrave.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/transform_listener.h>

class TfPlugin : public OpenRAVE::SensorBase {
public:
    TfPlugin(OpenRAVE::EnvironmentBasePtr env, std::string const &openrave_tf_frame);
    virtual ~TfPlugin(void);

    virtual void Reset(void);
    virtual void Destroy(void);
    virtual bool SimulationStep(OpenRAVE::dReal fTimeElapsed);

    virtual bool Init(const std::string& cmd);
    virtual bool Connect(std::ostream &output, std::istream &input);
    virtual void Reset(int options);
    virtual int Configure(OpenRAVE::SensorBase::ConfigureCommand command, bool blocking=false);
    virtual OpenRAVE::SensorBase::SensorGeometryPtr GetSensorGeometry(OpenRAVE::SensorBase::SensorType type=OpenRAVE::SensorBase::ST_Invalid);
    virtual OpenRAVE::SensorBase::SensorDataPtr CreateSensorData(OpenRAVE::SensorBase::SensorType type=OpenRAVE::SensorBase::ST_Invalid);
    virtual bool GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata);
    virtual bool Supports(OpenRAVE::SensorBase::SensorType type);
    virtual OpenRAVE::Transform GetTransform();
    virtual void SetTransform(OpenRAVE::Transform const &transform);

    bool RegisterBody(std::ostream& sout, std::istream& sinput);
    bool UnregisterBody(std::ostream& sout, std::istream& sinput);
    bool Pause(std::ostream& sout, std::istream& sinput);
    bool Resume(std::ostream& sout, std::istream& sinput);
    bool Clear(std::ostream& sout, std::istream& sinput);

    bool RegisterBodyHelper(std::string const &body_name, std::string const &tf_frame);
    bool UnregisterBodyHelper(std::string const &body_name);
private:
    std::string openrave_tf_frame_;

    ros::NodeHandle nh_;
    tf::TransformListener tf_;

    boost::mutex mutex_;
    std::map<std::string, std::string> bodies_;
    bool paused_;

    bool IsBodyRegistered(std::string const &body_name);
    OpenRAVE::Transform GetOrTransform(tf::StampedTransform const &transform);

};

#endif // TFPLUGIN_H_

