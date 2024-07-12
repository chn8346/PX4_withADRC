#ifndef ADRC_STATUS_Z_STREAM_HPP
#define ADRC_STATUS_Z_STREAM_HPP

#include <uORB/topics/ladrc_status.h>
//#include <mavlink.h>
//#include <mavlink_stream.h>

class MavlinkStreamAdrcStatusZ: public MavlinkStream
{
public:
    const char *get_name() const { return MavlinkStreamAdrcStatusZ::get_name_static(); }

    static const char *get_name_static() { return "ADRC_STATUS_Z"; }

    static uint16_t get_id_static(){ return MAVLINK_MSG_ID_ADRC_STATUS_Z; }

    uint16_t get_id(){ return get_id_static(); }

    static MavlinkStream *new_instance(Mavlink *mavlink){ return new MavlinkStreamAdrcStatusZ(mavlink); }

    unsigned get_size(){ return MAVLINK_MSG_ID_ADRC_STATUS_Z_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    uORB::Subscription _sub_z{ORB_ID(ladrc_status_ratez)};

    /* do not allow top copying this class */
    MavlinkStreamAdrcStatusZ(MavlinkStreamAdrcStatusZ  &);
    MavlinkStreamAdrcStatusZ& operator = (const MavlinkStreamAdrcStatusZ  &);

protected:
    explicit MavlinkStreamAdrcStatusZ(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
	ladrc_status_s ladrc_status_uorb_z;    // uORB topic rate z

	if (_sub_z.update(&ladrc_status_uorb_z)) {
	    __mavlink_adrc_status_z_t ladrc_status_mavlink;  //make sure mavlink is the definition of your custom MAVLink message

	    ladrc_status_mavlink.v = ladrc_status_uorb_z.v;
	    ladrc_status_mavlink.v1 = ladrc_status_uorb_z.v1;
	    ladrc_status_mavlink.v2 = ladrc_status_uorb_z.v2;

	    ladrc_status_mavlink.e1 = ladrc_status_uorb_z.e1;
	    ladrc_status_mavlink.e2 = ladrc_status_uorb_z.e2;

	    ladrc_status_mavlink.u0 = ladrc_status_uorb_z.u0;
	    ladrc_status_mavlink.u = ladrc_status_uorb_z.u;

	    ladrc_status_mavlink.y = ladrc_status_uorb_z.y;

	    ladrc_status_mavlink.z1 = -ladrc_status_uorb_z.z1;
	    ladrc_status_mavlink.z2 = -ladrc_status_uorb_z.z2;
	    ladrc_status_mavlink.z3 = -ladrc_status_uorb_z.z3;

	    mavlink_msg_adrc_status_z_send_struct(_mavlink->get_channel(), &ladrc_status_mavlink);

	    return true;
	}

        return true;
    }
};

#endif // ADRC_STATUS_Z_STREAM_HPP
