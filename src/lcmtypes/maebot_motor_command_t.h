// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifndef _maebot_motor_command_t_h
#define _maebot_motor_command_t_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _maebot_motor_command_t maebot_motor_command_t;
struct _maebot_motor_command_t
{
    int64_t    utime;
    float      motor_left_speed;
    float      motor_right_speed;
};

/**
 * Create a deep copy of a maebot_motor_command_t.
 * When no longer needed, destroy it with maebot_motor_command_t_destroy()
 */
maebot_motor_command_t* maebot_motor_command_t_copy(const maebot_motor_command_t* to_copy);

/**
 * Destroy an instance of maebot_motor_command_t created by maebot_motor_command_t_copy()
 */
void maebot_motor_command_t_destroy(maebot_motor_command_t* to_destroy);

/**
 * Identifies a single subscription.  This is an opaque data type.
 */
typedef struct _maebot_motor_command_t_subscription_t maebot_motor_command_t_subscription_t;

/**
 * Prototype for a callback function invoked when a message of type
 * maebot_motor_command_t is received.
 */
typedef void(*maebot_motor_command_t_handler_t)(const lcm_recv_buf_t *rbuf,
             const char *channel, const maebot_motor_command_t *msg, void *userdata);

/**
 * Publish a message of type maebot_motor_command_t using LCM.
 *
 * @param lcm The LCM instance to publish with.
 * @param channel The channel to publish on.
 * @param msg The message to publish.
 * @return 0 on success, <0 on error.  Success means LCM has transferred
 * responsibility of the message data to the OS.
 */
int maebot_motor_command_t_publish(lcm_t *lcm, const char *channel, const maebot_motor_command_t *msg);

/**
 * Subscribe to messages of type maebot_motor_command_t using LCM.
 *
 * @param lcm The LCM instance to subscribe with.
 * @param channel The channel to subscribe to.
 * @param handler The callback function invoked by LCM when a message is received.
 *                This function is invoked by LCM during calls to lcm_handle() and
 *                lcm_handle_timeout().
 * @param userdata An opaque pointer passed to @p handler when it is invoked.
 * @return 0 on success, <0 if an error occured
 */
maebot_motor_command_t_subscription_t* maebot_motor_command_t_subscribe(lcm_t *lcm, const char *channel, maebot_motor_command_t_handler_t handler, void *userdata);

/**
 * Removes and destroys a subscription created by maebot_motor_command_t_subscribe()
 */
int maebot_motor_command_t_unsubscribe(lcm_t *lcm, maebot_motor_command_t_subscription_t* hid);

/**
 * Sets the queue capacity for a subscription.
 * Some LCM providers (e.g., the default multicast provider) are implemented
 * using a background receive thread that constantly revceives messages from
 * the network.  As these messages are received, they are buffered on
 * per-subscription queues until dispatched by lcm_handle().  This function
 * how many messages are queued before dropping messages.
 *
 * @param subs the subscription to modify.
 * @param num_messages The maximum number of messages to queue
 *  on the subscription.
 * @return 0 on success, <0 if an error occured
 */
int maebot_motor_command_t_subscription_set_queue_capacity(maebot_motor_command_t_subscription_t* subs,
                              int num_messages);

/**
 * Encode a message of type maebot_motor_command_t into binary form.
 *
 * @param buf The output buffer.
 * @param offset Encoding starts at this byte offset into @p buf.
 * @param maxlen Maximum number of bytes to write.  This should generally
 *               be equal to maebot_motor_command_t_encoded_size().
 * @param msg The message to encode.
 * @return The number of bytes encoded, or <0 if an error occured.
 */
int maebot_motor_command_t_encode(void *buf, int offset, int maxlen, const maebot_motor_command_t *p);

/**
 * Decode a message of type maebot_motor_command_t from binary form.
 * When decoding messages containing strings or variable-length arrays, this
 * function may allocate memory.  When finished with the decoded message,
 * release allocated resources with maebot_motor_command_t_decode_cleanup().
 *
 * @param buf The buffer containing the encoded message
 * @param offset The byte offset into @p buf where the encoded message starts.
 * @param maxlen The maximum number of bytes to read while decoding.
 * @param msg Output parameter where the decoded message is stored
 * @return The number of bytes decoded, or <0 if an error occured.
 */
int maebot_motor_command_t_decode(const void *buf, int offset, int maxlen, maebot_motor_command_t *msg);

/**
 * Release resources allocated by maebot_motor_command_t_decode()
 * @return 0
 */
int maebot_motor_command_t_decode_cleanup(maebot_motor_command_t *p);

/**
 * Check how many bytes are required to encode a message of type maebot_motor_command_t
 */
int maebot_motor_command_t_encoded_size(const maebot_motor_command_t *p);
size_t maebot_motor_command_t_struct_size(void);
int  maebot_motor_command_t_num_fields(void);
int  maebot_motor_command_t_get_field(const maebot_motor_command_t *p, int i, lcm_field_t *f);
const lcm_type_info_t *maebot_motor_command_t_get_type_info(void);

// LCM support functions. Users should not call these
int64_t __maebot_motor_command_t_get_hash(void);
int64_t __maebot_motor_command_t_hash_recursive(const __lcm_hash_ptr *p);
int     __maebot_motor_command_t_encode_array(void *buf, int offset, int maxlen, const maebot_motor_command_t *p, int elements);
int     __maebot_motor_command_t_decode_array(const void *buf, int offset, int maxlen, maebot_motor_command_t *p, int elements);
int     __maebot_motor_command_t_decode_array_cleanup(maebot_motor_command_t *p, int elements);
int     __maebot_motor_command_t_encoded_array_size(const maebot_motor_command_t *p, int elements);
int     __maebot_motor_command_t_clone_array(const maebot_motor_command_t *p, maebot_motor_command_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
