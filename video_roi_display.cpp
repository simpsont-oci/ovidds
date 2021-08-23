#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <mutex>

#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include "dds/DCPS/LocalObject.h"
#include <dds/DCPS/transport/framework/TransportRegistry.h>
#include "dds/DCPS/RTPS/RtpsDiscovery.h"
#include "dds/DCPS/transport/rtps_udp/RtpsUdp.h"

#include "ovidds_common.h"
#include "oviddsTypeSupportImpl.h"

using namespace std;
using namespace cv;

typedef DDS::DomainParticipantFactory_var factory_type;
typedef DDS::DomainParticipant_var part_type;;
typedef DDS::DomainParticipantQos part_qos_type;
typedef DDS::DataReader_var dr_type;

class datareader_listener : public virtual OpenDDS::DCPS::LocalObject<DDS::DataReaderListener>
{
public:

  datareader_listener()
    : reliable_(is_reliable())
    , new_frame_(false)
  {
    std::cout << "Transport is " << (reliable_ ? "" : "UN-") << "RELIABLE" <<  std::endl;
  }

  virtual ~datareader_listener() {}

  virtual void on_requested_deadline_missed(DDS::DataReader_ptr, const DDS::RequestedDeadlineMissedStatus&)
  {
    std::cout << "datareader_listener::on_requested_deadline_missed()" << std::endl;
  }

  virtual void on_requested_incompatible_qos(
    DDS::DataReader_ptr,
    const DDS::RequestedIncompatibleQosStatus&)
  {
    std::cout << "datareader_listener::on_requested_incompatible_qos()" << std::endl;
  }

  virtual void on_liveliness_changed(
    DDS::DataReader_ptr,
    const DDS::LivelinessChangedStatus&)
  {
    std::cout << "datareader_listener::on_liveliness_changed()" << std::endl;
  }

  virtual void on_subscription_matched(
    DDS::DataReader_ptr,
    const DDS::SubscriptionMatchedStatus& status)
  {
    std::cout << "datareader_listener::on_subscription_matched() " << status.current_count << std::endl;
  }

  virtual void on_sample_rejected(
    DDS::DataReader_ptr,
    const DDS::SampleRejectedStatus&)
  {
    std::cout << "datareader_listener::on_sample_rejected()" << std::endl;
  }

  virtual void on_data_available(
    DDS::DataReader_ptr reader)
  {
    //std::cout << "datareader_listener::on_data_available()" << std::endl;
    ovidds::FrameDataReader_var frame_dr = ovidds::FrameDataReader::_narrow(reader);

    if (CORBA::is_nil(frame_dr.in())) {
      std::cerr << "unable to narrow reader" << std::endl;
      return;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    DDS::ReturnCode_t status = frame_dr->take_next_sample(frame_, si_);
    new_frame_ = status == DDS::RETCODE_OK;
  }

  virtual void on_sample_lost(
    DDS::DataReader_ptr,
    const DDS::SampleLostStatus&)
  {
    std::cout << "datareader_listener::on_sample_lost()" << std::endl;
  }

  bool get_image(Mat& mat, std::string& name)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (new_frame_)
    {
      new_frame_ = false;
      if (si_.valid_data)
      {
        std::stringstream ss;
        ss << "Video Source " << frame_.source_id << std::flush;

        name = ss.str();

        mat = Mat(frame_.size_y, frame_.size_x, frame_.type, &frame_.data[0]).clone();
        return true;
      }
      return false;
    }
    return false;
  }

private:

  bool reliable_;
  bool new_frame_;
  ovidds::Frame frame_;
  DDS::SampleInfo si_;
  std::mutex mutex_;
};

int main(int argc, char** argv)
{
  bool run = true;

  factory_type pf = TheParticipantFactoryWithArgs(argc, argv);

  part_qos_type part_qos;
  pf->get_default_participant_qos(part_qos);

  part_type part = pf->create_participant(DOMAIN,
                                          part_qos,
                                          DDS::DomainParticipantListener::_nil(),
                                          OpenDDS::DCPS::DEFAULT_STATUS_MASK);
  if (CORBA::is_nil(part.in())) {
    std::cerr << "Failed to create participant." << std::endl;
    return -1;
  }

  ovidds::FrameTypeSupport_var fts(new ovidds::FrameTypeSupportImpl());

  if (fts->register_type(part.in(), "") != DDS::RETCODE_OK) {
    std::cerr << "Failed to register ovidds message type with participant." << std::endl;
    return -1;
  }

  // Create Topic
  CORBA::String_var type_name = fts->get_type_name();
  DDS::Topic_var topic = part->create_topic(FRAME_TOPIC_NAME,
                                            type_name.in(),
                                            TOPIC_QOS_DEFAULT,
                                            DDS::TopicListener::_nil(),
                                            OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  if (CORBA::is_nil(topic.in())) {
    std::cerr << "Failed to create topic." << std::endl;
    return -1;
  }

  // Create Subscriber
  DDS::Subscriber_var sub = part->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                                    DDS::SubscriberListener::_nil(),
                                                    OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  if (CORBA::is_nil(sub.in())) {
    std::cerr << "Failed to create subscriber." << std::endl;
    return -1;
  }

  // Create DataReader
  DDS::DataReaderQos dr_qos;
  sub->get_default_datareader_qos(dr_qos);
  if (is_reliable()) {
    std::cout << "Create Reliable DataReader" << std::endl;
    dr_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
  }

  datareader_listener* listener = new datareader_listener();
  DDS::DataReaderListener_var dr_listener(listener);

  DDS::DataReader_var dr = sub->create_datareader(topic.in(),
                                                  dr_qos,
                                                  dr_listener.in(),
                                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  Mat image;
  std::string name;

  while (run)
  {
    if (listener->get_image(image, name))
    {
      imshow(name, image);
    }

    char c = static_cast<char>(waitKey(10));
  
    // Press q to exit from window
    if (c == 27 || c == 'q' || c == 'Q') 
      run = false;
  }

  part->delete_contained_entities();
  pf->delete_participant(part.in());
  TheServiceParticipant->shutdown();

  return 0;
}

