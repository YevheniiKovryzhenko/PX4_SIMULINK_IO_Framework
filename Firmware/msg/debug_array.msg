uint8 SIMULINK_INBOUND_ID = 200
uint8 SIMULINK_OUTBOUND_ID = 100

uint8 ARRAY_SIZE = 58
uint64 timestamp            # time since system start (microseconds)
uint16 id                   # unique ID of debug array, used to discriminate between arrays
char[10] name               # name of the debug array (max. 10 characters)
float32[58] data            # data

# TOPICS debug_array simulink_inbound simulink_outbound
