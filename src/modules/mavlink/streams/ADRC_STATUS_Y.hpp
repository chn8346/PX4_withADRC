#ifndef ADRC_STATUS_Y_STREAM_HPP
#define ADRC_STATUS_Y_STREAM_HPP

#include <uORB/topics/ladrc_status.h>
//#include <mavlink.h>
//#include <mavlink_stream.h>

class MavlinkStreamAdrcStatusY: public MavlinkStream
{
public:
    const char *get_name() const { return MavlinkStreamAdrcStatusY::get_name_static(); }

    static const char *get_name_static() { return "ADRC_STATUS_Y"; }

    static uint16_t get_id_static(){ return MAVLINK_MSG_ID_ADRC_STATUS_Y; }

    uint16_t get_id(){ return get_id_static(); }

    static MavlinkStream *new_instance(Mavlink *mavlink){ return new MavlinkStreamAdrcStatusY(mavlink); }

    unsigned get_size(){ return MAVLINK_MSG_ID_ADRC_STATUS_Y_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    uORB::Subscription _sub_y{ORB_ID(ladrc_status_ratey)};

    /* do not allow top copying this class */
    MavlinkStreamAdrcStatusY(MavlinkStreamAdrcStatusY &);
    MavlinkStreamAdrcStatusY& operator = (const MavlinkStreamAdrcStatusY &);

protected:
    explicit MavlinkStreamAdrcStatusY(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
	ladrc_status_s ladrc_status_uorb_y;    // uORB topic rate y

	if (_sub_y.update(&ladrc_status_uorb_y)) {
	    __mavlink_adrc_status_y_t ladrc_status_mavlink;  //make sure mavlink is the definition of your custom MAVLink message

	    ladrc_status_mavlink.v = ladrc_status_uorb_y.v;
	    ladrc_status_mavlink.v1 = ladrc_status_uorb_y.v1;
	    ladrc_status_mavlink.v2 = ladrc_status_uorb_y.v2;

	    ladrc_status_mavlink.e1 = ladrc_status_uorb_y.e1;
	    ladrc_status_mavlink.e2 = ladrc_status_uorb_y.e2;

	    ladrc_status_mavlink.u0 = ladrc_status_uorb_y.u0;
	    ladrc_status_mavlink.u = ladrc_status_uorb_y.u;

	    ladrc_status_mavlink.y = ladrc_status_uorb_y.y;

	    ladrc_status_mavlink.z1 = -ladrc_status_uorb_y.z1;
	    ladrc_status_mavlink.z2 = -ladrc_status_uorb_y.z2;
	    ladrc_status_mavlink.z3 = -ladrc_status_uorb_y.z3;

	    mavlink_msg_adrc_status_y_send_struct(_mavlink->get_channel(), &ladrc_status_mavlink);

	    return true;
	}

        return true;
    }
};

#endif // ADRC_STATUS_Y_STREAM_HPP
