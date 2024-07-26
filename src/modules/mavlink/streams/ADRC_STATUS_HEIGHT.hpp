#ifndef ADRC_STATUS_HEIGHT_STREAM_HPP
#define ADRC_STATUS_HEIGHT_STREAM_HPP

#include <uORB/topics/ladrc_status.h>
//#include <mavlink.h>
//#include <mavlink_stream.h>

class MavlinkStreamAdrcStatusHeight: public MavlinkStream
{
public:
    const char *get_name() const { return MavlinkStreamAdrcStatusHeight::get_name_static(); }

    static const char *get_name_static() { return "ADRC_STATUS_HEIGHT"; }

    static uint16_t get_id_static(){ return MAVLINK_MSG_ID_ADRC_STATUS_HEIGHT; }

    uint16_t get_id(){ return get_id_static(); }

    static MavlinkStream *new_instance(Mavlink *mavlink){ return new MavlinkStreamAdrcStatusHeight(mavlink); }

    unsigned get_size(){ return MAVLINK_MSG_ID_ADRC_STATUS_HEIGHT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    uORB::Subscription _sub_height{ORB_ID(ladrc_status_height_v)};

    /* do not allow top copying this class */
    MavlinkStreamAdrcStatusHeight(MavlinkStreamAdrcStatusHeight  &);
    MavlinkStreamAdrcStatusHeight& operator = (const MavlinkStreamAdrcStatusHeight  &);

protected:
    explicit MavlinkStreamAdrcStatusHeight(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
	ladrc_status_s ladrc_status_uorb_height;    // uORB topic rate in position Z-Axis

	if (_sub_height.update(&ladrc_status_uorb_height)) {
	    __mavlink_adrc_status_height_t ladrc_status_mavlink;  //make sure mavlink is the definition of your custom MAVLink message

	    ladrc_status_mavlink.v = ladrc_status_uorb_height.v;
	    ladrc_status_mavlink.v1 = ladrc_status_uorb_height.v1;
	    ladrc_status_mavlink.v2 = ladrc_status_uorb_height.v2;

	    ladrc_status_mavlink.e1 = ladrc_status_uorb_height.e1;
	    ladrc_status_mavlink.e2 = ladrc_status_uorb_height.e2;

	    ladrc_status_mavlink.u0 = ladrc_status_uorb_height.u0;
	    ladrc_status_mavlink.u = ladrc_status_uorb_height.u;

	    ladrc_status_mavlink.y = ladrc_status_uorb_height.y;

	    ladrc_status_mavlink.z1 = -ladrc_status_uorb_height.z1;
	    ladrc_status_mavlink.z2 = -ladrc_status_uorb_height.z2;
	    ladrc_status_mavlink.z3 = -ladrc_status_uorb_height.z3;

	    mavlink_msg_adrc_status_height_send_struct(_mavlink->get_channel(), &ladrc_status_mavlink);

	    return true;
	}

        return true;
    }
};

#endif // ADRC_STATUS_HEIGHT_STREAM_HPP
