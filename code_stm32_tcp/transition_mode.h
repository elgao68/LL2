#ifndef TRANSITION_MODE_H
#define TRANSITION_MODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t time;
        uint8_t mode;
        uint8_t index;
        uint64_t start_time_ms;
    } hman_transition_mode_params_t;

    enum
    {
        SINGLE_ITERATION = 0,
        CONTINUOUS_ITERATION = 1
    };

    void clear_transition_mode_params(void);
    uint8_t get_transition_mode_index(void);

    /*
	@brief:	set parameter related to transition time
	@param[in]: transition_time =  Transition mode enabled if non-zero "transition time"
	@param[in]: mode = 0 for SINGLE_ITERATION, 1 for CONTINUOUS_ITERATION
	@param[in]: restart = 1 start iteration from first target
    */
    void set_transition_mode_params(const uint32_t transition_time, const uint8_t mode, const uint8_t restart);
    bool is_transition_mode_active(void);
    void update_transition_mode(const uint64_t current_time_ms, const uint8_t target_params_size);

#ifdef __cplusplus
}
#endif

#endif
