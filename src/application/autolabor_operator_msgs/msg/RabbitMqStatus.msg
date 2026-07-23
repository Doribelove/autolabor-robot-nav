std_msgs/Header header

bool connected
string connection_state

string broker_host
uint32 broker_port
string virtual_host
string queue_name
string exchange_name
string routing_key

uint32 ready_message_count
uint32 consumer_count
uint64 received_message_count
uint64 accepted_message_count
uint64 rejected_message_count

time last_message_stamp
string last_error
bool has_cached_target
