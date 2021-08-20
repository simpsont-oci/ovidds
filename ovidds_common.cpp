#include "ovidds_common.h"

#include <dds/DCPS/transport/framework/TransportRegistry.h>

const long DOMAIN = 4;
const char* FRAME_TOPIC_NAME = "ovidds_frames";

bool is_reliable() {
  OpenDDS::DCPS::TransportConfig_rch gc = TheTransportRegistry->global_config();
  return !(gc->instances_[0]->transport_type_ == "udp");
}

