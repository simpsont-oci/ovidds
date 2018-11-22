#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <thread>

#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include "dds/DCPS/LocalObject.h"

#include "ovidds_common.h"
#include "oviddsTypeSupportImpl.h"

using namespace std;
using namespace cv;

typedef DDS::DataWriter_var dw_type;
typedef DDS::DomainParticipantFactory_var factory_type;
typedef DDS::DomainParticipant_var part_type;;
typedef DDS::DomainParticipantQos part_qos_type;

const uint16_t SizeX = 1280;
const uint16_t SizeY = 720;

class datawriter_listener : public virtual OpenDDS::DCPS::LocalObject<DDS::DataWriterListener>
{
public:

  explicit datawriter_listener(bool& connected)
    : reliable_(is_reliable())
    , connected_(connected)
  {
    std::cout << "Transport is " << (reliable_ ? "" : "UN-") << "RELIABLE" <<  std::endl;
  }

  virtual ~datawriter_listener() {}

  virtual void on_publication_matched(
    DDS::DataWriter_ptr,
    const DDS::PublicationMatchedStatus& status)
  {
    std::cout << "datawriter_listener::on_publication_matched() " << status.current_count << std::endl;
    connected_ = status.current_count != 0;
  }

  virtual void on_offered_deadline_missed(DDS::DataWriter_ptr, const DDS::OfferedDeadlineMissedStatus&)
  {
    std::cout << "datawriter_listener::on_offered_deadline_missed() " << std::endl;
  }

  virtual void on_offered_incompatible_qos(DDS::DataWriter_ptr, const DDS::OfferedIncompatibleQosStatus&)
  {
    std::cout << "datawriter_listener::on_offered_incompatible_qos() " << std::endl;
  }

  virtual void on_liveliness_lost(DDS::DataWriter_ptr, const DDS::LivelinessLostStatus&)
  {
    std::cout << "datawriter_listener::on_liveliness_lost() " << std::endl;
  }

private:

  bool reliable_;
  bool& connected_;
};

void write_thread(const bool& run, const bool& connected, dw_type& dw, VideoCapture& capture)
{
  Mat ReferenceFrame;

  // Write samples
  ovidds::FrameDataWriter_var frame_dw = ovidds::FrameDataWriter::_narrow(dw.in());

  if (CORBA::is_nil(frame_dw.in())) {
    std::cerr << "Failed to narrow datawriter for frames." << std::endl;
    return;
  }

  std::stringstream ss;
  size_t thread_id;
  ss << std::this_thread::get_id() << std::flush;
  ss >> thread_id;
  std::cout << "Writer thread ID: '" << ss.str() << "' read as size_t: 0x" << std::hex << thread_id << std::dec << std::endl;

  ovidds::Frame frame;

  frame.source_id = static_cast<long>(thread_id);
  std::cout << "Using frame source ID: 0x" << std::hex << frame.source_id << std::dec << std::endl;

  frame.count = 0;
  frame.type = 0;
  frame.format = 0;
  frame.size_x = 0;
  frame.size_y = 0;

  DDS::InstanceHandle_t handle = frame_dw->register_instance(frame);
  
  while (run)
  {
    if (connected)
    {
      capture >> ReferenceFrame;

      ++frame.count;

      if (frame.size_x != ReferenceFrame.cols ||
          frame.size_y != ReferenceFrame.rows ||
          frame.data.length() != ReferenceFrame.total() * ReferenceFrame.elemSize() ||
          frame.type != ReferenceFrame.type())
      {
        std::cout << "New captured frame of type " << ReferenceFrame.type() << " with size: " << ReferenceFrame.cols << " x " << ReferenceFrame.rows << " and "
          << ReferenceFrame.channels() << " channels. (element size = " << ReferenceFrame.elemSize() << ")" << std::endl;
      }

      frame.type = ReferenceFrame.type();

      frame.size_x = ReferenceFrame.cols;
      frame.size_y = ReferenceFrame.rows;

      frame.data.length(ReferenceFrame.total() * ReferenceFrame.elemSize());
      std::memcpy(&frame.data[0], ReferenceFrame.datastart, ReferenceFrame.total() * ReferenceFrame.elemSize());

      if (ReferenceFrame.cols > 3840 || ReferenceFrame.rows > 2160) {
        std::cout << "Frame dimensions of " << ReferenceFrame.cols << " x " << ReferenceFrame.rows << " don't fit within a UHD (3840 x 2160) screen, ignoring." << std::endl;
        continue;
      }

      DDS::ReturnCode_t error = DDS::RETCODE_OK;
      do {
        error = frame_dw->write(frame, handle);
      } while (connected && error == DDS::RETCODE_TIMEOUT);

      if (error != DDS::RETCODE_OK) {
        std::cerr << "Failed to write frame." << std::endl;
      }
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  frame_dw->unregister_instance(frame, handle);
}

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

  // Create Publisher
  DDS::Publisher_var pub = part->create_publisher(PUBLISHER_QOS_DEFAULT,
                                                  DDS::PublisherListener::_nil(),
                                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  if (CORBA::is_nil(pub.in())) {
    std::cerr << "Failed to create publisher." << std::endl;
    return -1;
  }

  DDS::DataWriterQos qos;
  pub->get_default_datawriter_qos(qos);
  if (is_reliable()) {
    std::cerr << "Reliable DataWriter" << std::endl;
    qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    //qos.lifespan.duration.sec = 0;
    //qos.lifespan.duration.nanosec = 1e8; // This would be about 10 frames a second
  }

  // Create DataWriter
  bool connected = false;

  DDS::DataWriterListener_var dw_listener(new datawriter_listener(connected));

  DDS::DataWriter_var dw = pub->create_datawriter(topic.in(),
                                                  qos,
                                                  dw_listener,
                                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  if (CORBA::is_nil(dw.in())) {
    std::cerr << "Failed to create datawriter." << std::endl;
    return -1;
  }

  if (dw == NULL)
  {
    std::cerr << "Error: Cannot create OpenDDS datawriter" << std::endl;
    return -1;
  }

  VideoCapture capture;

  capture.open(0);

  if (!capture.isOpened())
  {
    std::cerr << "Error: Cannot open video stream from camera" << std::endl;
    return -1;
  }

  if (!capture.set(CAP_PROP_FRAME_HEIGHT, SizeY))
  {
    std::cerr << "Error: Cannot resize height to " << SizeY << std::endl;
  }

  if (!capture.set(CAP_PROP_FRAME_WIDTH, SizeX))
  {
    std::cerr << "Error: Cannot resize width to " << SizeX << std::endl;
  }

  /*if (!capture.set(CAP_PROP_MODE, CAP_MODE_YUYV))
  {
    std::cerr << "Error: Cannot set capture mode." << std::endl;
  }*/

  /*if (!capture.set(CAP_PROP_MONOCHROME, false))
  {
    std::cerr << "Error: Cannot set capture mode." << std::endl;
  }*/

  /*if (!capture.set(CAP_PROP_CONVERT_RGB, false))
  {
    std::cerr << "Error: Unable to turn off RGB conversion " << std::endl;
  }*/

  std::thread writer(std::bind(write_thread, std::cref(run), std::cref(connected), std::ref(dw), std::ref(capture)));

  std::string line;
  std::getline(std::cin, line);
  run = false;

  writer.join();

  part->delete_contained_entities();
  pf->delete_participant(part.in());
  TheServiceParticipant->shutdown();

  return 0;
}

