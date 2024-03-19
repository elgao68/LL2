#include "transition_mode.h"
#include <string.h>

static hman_transition_mode_params_t hman_transition_mode_params;

void clear_transition_mode_params(void)
{
    memset(&hman_transition_mode_params, 0, sizeof(hman_transition_mode_params_t));
}

uint8_t get_transition_mode_index(void)
{
    return hman_transition_mode_params.index;
}

/*
	@brief:	set parameter related to transition time
	@param[in]: transition_time =  Transition mode enabled if non-zero "transition time"
	@param[in]: mode = 0 for SINGLE_ITERATION, 1 for CONTINUOUS_ITERATION
    @param[in]: restart = 1 start iteration from first target
*/
void set_transition_mode_params(const uint32_t transition_time, const uint8_t mode, const uint8_t restart)
{
    if(restart == 1) //Start from first target
        clear_transition_mode_params();
    hman_transition_mode_params.time = transition_time;
    hman_transition_mode_params.mode = mode;
}

bool is_transition_mode_active(void)
{
    return (hman_transition_mode_params.time != 0);
}

void update_transition_mode(const uint64_t current_time_ms, const uint8_t target_params_size)
{
    if (is_transition_mode_active())
    {
        if (hman_transition_mode_params.start_time_ms == 0)
            hman_transition_mode_params.start_time_ms = current_time_ms;

        // if elapsed time > transition time
        if ((current_time_ms - hman_transition_mode_params.start_time_ms) >= hman_transition_mode_params.time)
        {
            //check for mode
            switch (hman_transition_mode_params.mode)
            {
            case SINGLE_ITERATION:
                if ((hman_transition_mode_params.index + 1) < target_params_size)
                {
                    hman_transition_mode_params.index++;
                    hman_transition_mode_params.start_time_ms = current_time_ms;
                }
                break;

            case CONTINUOUS_ITERATION:
                hman_transition_mode_params.index = (hman_transition_mode_params.index + 1) % target_params_size;
                hman_transition_mode_params.start_time_ms = current_time_ms;
                break;

            default:
                break;
            }
        }
    }
}
